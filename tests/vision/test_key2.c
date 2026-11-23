#include <stdlib.h>
#include <time.h>
#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#endif
#define main static_regression_main
#include "test_static.c"
#undef main

enum { KEY2_META_WIRE = 12, KEY2_CHUNK_WIRE = 13 };

static unsigned zone_frame_id;
static float zone_x, zone_y, fixture_goal_y;

static void sample_pose(float world_x, float world_y, float yaw)
{
    float imu_raw = fmodf(yaw * 180.0f / RETURN_PI, 360.0f);
    imu_sample(imu_raw < 0 ? imu_raw + 360.0f : imu_raw);
    x_body = world_x - FIELD_START_X_CM;
    y_body = world_y - FIELD_START_Y_CM;
    angle_body = yaw;
    angle_imu = yaw; /* A directly injected encoder/IMU pose has a shared heading reference. */
    encoder1.last_tick = encoder2.last_tick = encoder3.last_tick = encoder4.last_tick = tick;
    field_map_record(tick, x_body, y_body, yaw);
}

static void zone_observation(uint32_t id, uint32_t captured, float actual_x,
                             float actual_y, float yaw, VisionKind kind)
{
    VisionObservation *upper = &vision.upper;
    VisionObject *o = &upper->objects[0];
    const float dx = zone_x - actual_x;
    const float dy = zone_y - actual_y;
    const float x = dx * cosf(yaw) - dy * sinf(yaw);
    const float y = dx * sinf(yaw) + dy * cosf(yaw);
    memset(upper, 0, sizeof(*upper));
    upper->frame_id = id;
    upper->updated_ms = tick;
    upper->age_ms = tick - captured;
    upper->available = upper->fresh = 1;
    upper->status = kind == VISION_NONE ? VISION_OBS_EMPTY : VISION_OBS_OK;
    if (kind == VISION_NONE) return;
    upper->count = 1;
    o->kind = kind;
    o->position_present = o->angle_present = o->distance_valid = 1;
    o->x_mm = (int32_t)lroundf(x * 10);
    o->y_mm = (int32_t)lroundf(y * 10);
    o->distance_mm = (int32_t)lroundf(hypotf(x, y) * 10);
    o->angle = atan2f(x, y) * 180.0f / RETURN_PI;
}

static void same_coordinate(float actual, float expected)
{
    assert(fabsf(actual - expected) < .12f);
}

static void key_command(unsigned id, unsigned key)
{
    char body[32], line[48];
    int length = snprintf(body, sizeof(body), "C1,%u,K,%u", id, key);
    assert(length > 0 && length < (int)sizeof(body));
    snprintf(line, sizeof(line), "%s*%04X\n", body, VSP_Crc16((const uint8_t *)body, (uint16_t)length));
    command(line);
}

static void key2_request(const char *request, unsigned type, WireFrame *response)
{
    const uint32_t deadline = tick + 3000;
    reset_wire();
    command(request);
    while (!wire_find(wire, wire_size, type, response)) {
        assert((int32_t)(deadline - tick) > 0);
        tick += 5;
        faraway_report();
        drain();
    }
}

static void key2_meta(WireFrame *meta, uint32_t nonce)
{
    char request[36];
    snprintf(request, sizeof(request), "KRUN?,%lu\n", (unsigned long)nonce);
    key2_request(request, KEY2_META_WIRE, meta);
    assert(meta->length == 28 && meta->payload[27] == 1);
}

static void emit_key2_wire(const char *label, unsigned type)
{
    unsigned offset = 0;
    WireFrame packet;
    while (wire_next(wire, wire_size, &offset, &packet)) {
        if (packet.type == type) {
            wire_vector(label, wire + offset - packet.encoded_size, packet.encoded_size - 1U);
            return;
        }
    }
    assert(0);
}

static void collect_session(const char *label, uint32_t nonce, int expected_complete, int emit)
{
    WireFrame meta, chunk;
    uint32_t boot, id, next_seq, dropped, start = 0, count = 0;
    uint8_t saw_kind[8] = {0};
    char request[96];
    key2_meta(&meta, nonce);
    boot = wire_u32(meta.payload);
    id = wire_u32(meta.payload + 4);
    next_seq = wire_u32(meta.payload + 8);
    dropped = wire_u32(meta.payload + 12);
    assert(boot && id && next_seq && (meta.payload[26] & 3U) == 2U);
    assert((dropped == 0) == expected_complete);
    if (emit) emit_key2_wire("key2-meta", KEY2_META_WIRE);
    while (start < next_seq) {
        const uint8_t *entry;
        unsigned i;
        uint32_t scan_next;
        snprintf(request, sizeof(request), "KREAD,%lu,%lu,%lu\n", (unsigned long)boot,
                 (unsigned long)id, (unsigned long)start);
        key2_request(request, KEY2_CHUNK_WIRE, &chunk);
        assert(chunk.length >= 17 && wire_u32(chunk.payload) == boot && wire_u32(chunk.payload + 4) == id);
        assert(wire_u32(chunk.payload + 8) == start);
        scan_next = wire_u32(chunk.payload + 12);
        assert(scan_next > start && scan_next <= next_seq);
        entry = chunk.payload + 17;
        for (i = 0; i < chunk.payload[16]; ++i) {
            uint32_t seq = wire_u32(entry);
            uint8_t kind = entry[8];
            assert(seq >= start && seq < scan_next && kind > 0 && kind < 8);
            if (kind == 1) assert(seq == 0);
            if (kind == 7) assert(seq == next_seq - 1);
            ++count;
            saw_kind[kind] = 1;
            entry += 10 + entry[9];
        }
        assert(entry == chunk.payload + chunk.length);
        if (emit) emit_key2_wire("key2-chunk", KEY2_CHUNK_WIRE);
        start = scan_next;
    }
    assert(count == wire_u16(meta.payload + 24) && count + dropped == next_seq);
    assert(saw_kind[1] && saw_kind[7]);
    if (emit) {
        for (unsigned i = 2; i < 7; ++i) assert(saw_kind[i]);
    }
    printf("PASS KEY2 %s: session=%lu records=%lu dropped=%lu START/END retained\n", label,
           (unsigned long)id, (unsigned long)count, (unsigned long)dropped);
}

static void geometry_and_keys(int emit)
{
    const float initial_x = zone_x - 20.0f;
    const float initial_y = fixture_goal_y + (zone_y < fixture_goal_y ? 42.0f : -42.0f);
    const float yaw = atan2f(zone_x - (initial_x + 3), zone_y - (initial_y - 2));
    const float aligned_yaw = zone_y < fixture_goal_y ? RETURN_PI : 0;
    uint32_t captured, good_id;
    FieldPose exposure;
    int16_t targets[4];
    unsigned i;

    memset(&vision, 0, sizeof(vision));
    field_map_reset(tick);
    sample_pose(initial_x, initial_y, yaw);
    if (emit) {
        key_press(key2_Pin);
    } else {
        reset_wire();
        key_command(1000, 2);
        assert(command_status == 0);
        key_command(1000, 2); /* Retransmit the same command number. */
        assert(command_status == 0);
    }
    assert(step == HUI_KU_TASK_STEP && hui_log_active());
    captured = tick;
    sample_pose(initial_x, initial_y, yaw);
    tick += 20;
    sample_pose(initial_x + 1, initial_y + 1, yaw);
    good_id = ++zone_frame_id;
    zone_observation(good_id, captured, initial_x + 3, initial_y - 2, yaw, return_zone->kind);
    assert(fabsf(vision.upper.objects[0].angle) < .01f);
    move();
    same_coordinate(field_map.robot.x, initial_x + 4);
    same_coordinate(field_map.robot.y, initial_y - 1);
    assert(fabsf(field_map.robot.yaw - yaw) < .001f && fabsf(body_motion.target_right) > 1.0f);
    assert(field_map_pose_at(captured, &exposure));
    same_coordinate(exposure.x, initial_x + 3);
    same_coordinate(exposure.y, initial_y - 2);
    printf("PASS KEY2 %s: %.1f,%.1f at capture, translated current to %.2f,%.2f, camera angle ~0, lateral %.2f cm/s\n",
           zone_y < fixture_goal_y ? "blue" : "red", initial_x, initial_y,
           field_map.robot.x, field_map.robot.y, body_motion.target_right);
    motor_process();
    Motor_GetWheelTargets(targets);
    assert(targets[0] || targets[1] || targets[2] || targets[3]);
    assert(abs(targets[0]) <= 900 && abs(targets[1]) <= 900 && abs(targets[2]) <= 900 && abs(targets[3]) <= 900);

    tick += 20;
    sample_pose(initial_x + 2, initial_y + 1, yaw);
    zone_observation(good_id, captured, initial_x + 10, initial_y - 8, yaw, return_zone->kind);
    move();
    same_coordinate(field_map.robot.x, initial_x + 5); /* Duplicate must not apply even if its content changes. */
    for (i = 0; i < 5; ++i) {
        tick += 20;
        sample_pose(initial_x + 2 + .2f * i, initial_y + 1, yaw);
        encoder1.count += 12;
        encoder2.count += 18;
        encoder3.count -= 9;
        encoder4.count -= 14;
        zone_observation(++zone_frame_id, tick, initial_x, initial_y, yaw, VISION_NONE);
        move();
        motor_process();
        assert(motor_drive_enabled && fabsf(body_motion.target_right) > 1.0f);
    }
    same_coordinate(field_map.robot.x, initial_x + 5.8f);
    puts("PASS KEY2: five missed frames, command remains active and pose follows odometry");

    captured = tick;
    tick += 20;
    sample_pose(initial_x + 3.8f, initial_y + 1, yaw);
    good_id = ++zone_frame_id;
    zone_observation(good_id, captured, initial_x + 6.8f, initial_y - 2, yaw, return_zone->kind);
    move();
    same_coordinate(field_map.robot.x, initial_x + 7.8f);
    same_coordinate(field_map.robot.y, initial_y - 2);
    move();
    same_coordinate(field_map.robot.x, initial_x + 7.8f);
    puts("PASS KEY2: first recovery frame applies one fix, including an overlapping delayed pose history");
    key_press(key2_Pin); /* Second press during this trip preserves the original log. */
    assert(hui_log_active());

    tick += 20;
    sample_pose(initial_x + 4, initial_y + 1, yaw);
    zone_observation(++zone_frame_id, tick, initial_x, initial_y, yaw,
                     zone_y < fixture_goal_y ? VISION_RED_MATERIAL_ZONE : VISION_BLUE_MATERIAL_ZONE);
    move();
    same_coordinate(field_map.robot.x, initial_x + 8);
    for (i = 0; i < 40; ++i) {
        tick += 20;
        sample_pose(initial_x + 4, initial_y + 1, yaw);
        move();
        motor_process();
    }
    assert(fabsf(body_motion.target_forward) < .01f && fabsf(body_motion.target_right) < .01f);
    puts("PASS KEY2: wrong team ignored, prolonged outage stops translation to reacquire");

    /* Put odometry at the survey point to test arrival. The control does not
       start a push or release routine, and must stay recorded until KEY4. */
    for (i = 0; i < 25; ++i) {
        tick += 20;
        sample_pose(zone_x - 4, fixture_goal_y + 3, aligned_yaw);
        if (i % 3 == 0)
            zone_observation(++zone_frame_id, tick, zone_x, fixture_goal_y, aligned_yaw, return_zone->kind);
        move();
        motor_process();
    }
    assert(hui_log_active() && hui_ku_stage() == HUI_COMPLETE);
    stopped();
    assert(servo_angle == 50);
    if (!emit) {
        /* Different source for START (remote) and END (physical). */
        const uint32_t end = tick;
        key_press(key4_Pin);
        assert(tick == end);
    } else key_press(key4_Pin);
    stopped();
    assert(!hui_log_active() && step == 0);
    collect_session("geometry", 311031, 1, emit);
}

static void estimated_zone_age_boundary(void)
{
    const float x = zone_x - 20.0f;
    const float y = fixture_goal_y;
    const float yaw = zone_y < fixture_goal_y ? RETURN_PI : 0.0f;
    uint32_t captured;

    memset(&vision, 0, sizeof(vision));
    field_map_reset(tick);
    sample_pose(x, y, yaw);
    key_press(key2_Pin);
    assert(step == HUI_KU_TASK_STEP && hui_log_active());

    captured = tick;
    tick += 199;
    sample_pose(x, y, yaw);
    zone_observation(++zone_frame_id, captured, x + 3, y - 2, yaw, return_zone->kind);
    vision.upper.objects[0].distance_valid = 0;
    move();
    assert(run.fixes == 1 && run.obs_reason == REASON_FIX);
    same_coordinate(field_map.robot.x, x + 3);
    same_coordinate(field_map.robot.y, y - 2);

    captured = tick;
    tick += 200;
    sample_pose(x, y, yaw);
    zone_observation(++zone_frame_id, captured, x + 8, y - 7, yaw, return_zone->kind);
    vision.upper.objects[0].distance_valid = 0;
    move();
    assert(run.fixes == 1 && run.obs_reason == REASON_STALE);
    same_coordinate(field_map.robot.x, x + 3);
    same_coordinate(field_map.robot.y, y - 2);

    tick += 20;
    sample_pose(x, y, yaw);
    zone_observation(++zone_frame_id, tick, x + 8, y - 7, yaw, return_zone->kind);
    vision.upper.objects[0].position_present = 0;
    move();
    assert(run.fixes == 1 && run.obs_reason == REASON_INVALID);

    tick += 20;
    sample_pose(x, y, yaw);
    zone_observation(++zone_frame_id, tick, x + 8, y - 7, yaw, return_zone->kind);
    vision.upper.objects[0].angle_present = 0;
    move();
    assert(run.fixes == 1 && run.obs_reason == REASON_INVALID);
    same_coordinate(field_map.robot.x, x + 3);
    same_coordinate(field_map.robot.y, y - 2);

    key_press(key4_Pin);
    stopped();
    puts("PASS KEY2: distance_valid=0 fixes pose at 199 ms, 200 ms rejected, XY and angle still required");
}

static void large_zone_fix_field_boundary(void)
{
    const int blue = zone_y < fixture_goal_y;
    const float yaw = blue ? 0.0f : RETURN_PI;
    const float inside_y = blue ? 2.0f : 298.0f;
    const float outside_y = blue ? -2.0f : 302.0f;
    const float correction = inside_y - fixture_goal_y;

    ++tick;
    memset(&vision, 0, sizeof(vision));
    field_map_reset(tick);
    sample_pose(zone_x, fixture_goal_y, yaw);
    key_press(key2_Pin);
    assert(step == HUI_KU_TASK_STEP && hui_log_active());

    tick += 20;
    sample_pose(zone_x, fixture_goal_y, yaw);
    zone_observation(++zone_frame_id, tick, zone_x, inside_y, yaw, return_zone->kind);
    move();
    assert(fabsf(correction) > 50.0f && run.fixes == 1 && run.obs_reason == REASON_FIX);
    same_coordinate(field_map.robot.x, zone_x);
    same_coordinate(field_map.robot.y, inside_y);

    tick += 20;
    sample_pose(zone_x, fixture_goal_y, yaw);
    zone_observation(++zone_frame_id, tick, zone_x, outside_y, yaw, return_zone->kind);
    move();
    assert(run.fixes == 1 && run.obs_reason == REASON_OUTLIER);
    same_coordinate(field_map.robot.x, zone_x);
    same_coordinate(field_map.robot.y, inside_y);

    key_press(key4_Pin);
    stopped();
    printf("PASS KEY2 field bounds: accepts in-field correction %.0f cm; rejects projected Y=%.0f cm\n",
           correction, outside_y);
}

static void terminal_assert_no_translation(void)
{
    assert(body_motion.target_forward == 0 && body_motion.target_right == 0);
}

static void terminal_setup(float x, float y, float yaw)
{
    ++tick;
    memset(&vision, 0, sizeof(vision));
    field_map_reset(tick);
    sample_pose(x, y, yaw);
    key_press(key2_Pin);
    assert(step == HUI_KU_TASK_STEP && hui_log_active());
}

typedef struct {
    unsigned long frame, captured, applied;
    float yaw, angle;
    long x_mm, y_mm;
} TerminalFix;

static void terminal_log_replay(const char *path)
{
    const float x = zone_x + .8f, y = fixture_goal_y;
    unsigned long start_tick, before_tick, after_tick, interrupted_frame;
    unsigned count, capture = 0, apply = 0, checked = 0, longest_gap = 0;
    uint32_t age_after = 0, begin;
    float yaw, turn_before = 0, turn_after = 0;
    FILE *fixture = fopen(path, "r");
    TerminalFix *fixes;
    assert(fixture);
    assert(fscanf(fixture, "%u %lu %lu %lu %lu", &count, &start_tick, &before_tick,
                  &after_tick, &interrupted_frame) == 5);
    assert(count > 1 && count <= 4096);
    fixes = calloc(count, sizeof(*fixes));
    assert(fixes);
    for (unsigned i = 0; i < count; ++i) {
        TerminalFix *fix = &fixes[i];
        assert(fscanf(fixture, "%lu %lu %lu %f %f %ld %ld", &fix->frame, &fix->captured,
                      &fix->applied, &fix->yaw, &fix->angle, &fix->x_mm, &fix->y_mm) == 7);
        assert(fix->captured <= fix->applied && fix->applied - fix->captured < HUI_OBS_MAX_MS);
        if (i) {
            assert(fixes[i - 1].captured <= fix->captured && fixes[i - 1].applied < fix->applied);
            if (fix->applied - fixes[i - 1].applied > longest_gap)
                longest_gap = (unsigned)(fix->applied - fixes[i - 1].applied);
        }
    }
    fclose(fixture);

    /* Keep the terminal position fixed, replay measured exposure headings and
     * camera XY/angles, and let the real controller run at its 20 ms cadence. */
    yaw = fixes[0].yaw * DEG_RAD;
    terminal_setup(x, y, yaw);
    begin = tick;
    for (unsigned long source_tick = fixes[0].captured;
         source_tick <= fixes[count - 1].applied + 20; ++source_tick) {
        uint8_t capture_pose = 0;
        const uint8_t control_cycle = source_tick % 20 == before_tick % 20;
        tick = begin + (uint32_t)(source_tick - fixes[0].captured);
        while (capture < count && source_tick == fixes[capture].captured) {
            yaw = fixes[capture++].yaw * DEG_RAD;
            capture_pose = 1;
        }
        if (capture_pose || control_cycle) sample_pose(x, y, yaw);
        if (apply < count && source_tick == fixes[apply].applied) {
            const TerminalFix *fix = &fixes[apply++];
            uint32_t exposed = begin + (uint32_t)(fix->captured - fixes[0].captured);
            zone_observation((uint32_t)fix->frame, exposed, x, y, yaw, return_zone->kind);
            vision.upper.objects[0].x_mm = (int32_t)fix->x_mm;
            vision.upper.objects[0].y_mm = (int32_t)fix->y_mm;
            vision.upper.objects[0].angle = fix->angle;
            vision.upper.objects[0].distance_valid = 0;
            hui_ku_observe_upper();
            assert(run.fixes == apply);
        }
        if (!control_cycle || !apply) continue;
        move();
        motor_process();
        if (hui_ku_stage() != HUI_CONFIRM || !motor_drive_enabled) {
            fprintf(stderr, "KEY2 terminal replay tick %lu frame %lu: stage=%d drive=%d reason=%u X=%.3f Y=%.3f bearing=%.3f\n",
                    source_tick, (unsigned long)run.good_frame, hui_ku_stage(), motor_drive_enabled,
                    run.reason, field_map.robot.x, field_map.robot.y, run.bearing);
        }
        assert(hui_ku_stage() == HUI_CONFIRM && motor_drive_enabled);
        terminal_assert_no_translation();
        assert(body_motion.target_turn < -5.0f);
        if (source_tick == before_tick || source_tick == after_tick) {
            assert(run.good_frame == interrupted_frame);
            if (source_tick == before_tick) turn_before = body_motion.target_turn;
            else {
                turn_after = body_motion.target_turn;
                age_after = tick - run.last_good_tick;
            }
        }
        ++checked;
    }
    assert(apply == count && turn_before < -16 && turn_before > -20 && age_after >= HUI_OBS_MAX_MS);
    assert(fabsf(turn_before - turn_after) < .2f);
    printf("PASS KEY2 terminal real timing: %.3f-%.3f s, %u fixes, max apply gap %u ms, %u continuous turn cycles without translation\n",
           (fixes[0].applied - start_tick) * .001, (fixes[count - 1].applied - start_tick) * .001,
           count, longest_gap, checked);
    printf("PASS KEY2 terminal frame %lu: at %.3f s turn %.2f deg/s; at %.3f s age %lu ms and turn %.2f deg/s\n",
           interrupted_frame, (before_tick - start_tick) * .001, turn_before,
           (after_tick - start_tick) * .001, (unsigned long)age_after, turn_after);
    key_press(key4_Pin);
    stopped();
    free(fixes);
}

static void terminal_yaw_confirmation(void)
{
    const float yaw = zone_y < fixture_goal_y ? RETURN_PI : 0.0f;
    const float initial_yaw = yaw + 7.06f * DEG_RAD;
    uint32_t captured;
    float raw_yaw;

    terminal_setup(zone_x, fixture_goal_y, initial_yaw);
    captured = tick;
    tick += 185;
    sample_pose(zone_x, fixture_goal_y, initial_yaw);
    zone_observation(++zone_frame_id, captured, zone_x, fixture_goal_y, initial_yaw, return_zone->kind);
    move();
    assert(hui_ku_stage() == HUI_CONFIRM && fabsf(body_motion.target_turn + 17.65f) < .1f);
    terminal_assert_no_translation();

    tick += 20;
    sample_pose(zone_x, fixture_goal_y, initial_yaw);
    move();
    assert(motor_drive_enabled && fabsf(body_motion.target_turn + 17.65f) < .1f);
    terminal_assert_no_translation();

    /* Simulate the latest IMU arriving one scheduler interval before the
     * corresponding encoder pose: the new angle alone must stop the turn. */
    tick += 20;
    raw_yaw = yaw / DEG_RAD;
    imu_sample(raw_yaw);
    move();
    assert(hui_ku_stage() == HUI_CONFIRM && fabsf(run.bearing) < .1f);
    stopped();

    tick += 20;
    sample_pose(zone_x, fixture_goal_y, yaw + 3.0f * DEG_RAD);
    move();
    assert(hui_ku_stage() == HUI_CONFIRM && fabsf(run.bearing) > HUI_ANGLE_TOL_DEG);
    stopped(); /* IMU drift alone cannot restart rotation after the stop latch. */

    encoder1.speed = 20000;
    for (unsigned i = 0; i < 15; ++i) {
        tick += 20;
        sample_pose(zone_x, fixture_goal_y, yaw);
        zone_observation(++zone_frame_id, tick, zone_x, fixture_goal_y, yaw, return_zone->kind);
        move();
        assert(hui_ku_stage() == HUI_CONFIRM);
        stopped();
    }
    encoder1.speed = 0;
    for (unsigned i = 0; i <= 10; ++i) {
        tick += 20;
        sample_pose(zone_x, fixture_goal_y, yaw);
        if (i < 10) zone_observation(++zone_frame_id, tick, zone_x, fixture_goal_y, yaw, VISION_NONE);
        move();
        assert(hui_ku_stage() == HUI_CONFIRM);
        stopped();
    }
    tick += 20;
    sample_pose(zone_x, fixture_goal_y, yaw);
    zone_observation(++zone_frame_id, tick, zone_x, fixture_goal_y, yaw, return_zone->kind);
    move();
    assert(hui_ku_stage() == HUI_COMPLETE);
    stopped();
    key_press(key4_Pin);
    puts("PASS KEY2 terminal simulated yaw: +7.06 deg heading error corrected by IMU, stopped in same cycle, waits for fresh fix and 200 ms settled wheels");
}

static void terminal_yaw_protection(void)
{
    const float yaw = (zone_y < fixture_goal_y ? RETURN_PI : 0.0f) + 7.06f * DEG_RAD;
    uint32_t captured;

    terminal_setup(zone_x, fixture_goal_y, yaw);
    captured = ++tick;
    sample_pose(zone_x, fixture_goal_y, yaw);
    zone_observation(++zone_frame_id, captured, zone_x, fixture_goal_y, yaw, return_zone->kind);
    move();
    assert(motor_drive_enabled);

    tick += 210;
    sample_pose(zone_x, fixture_goal_y, yaw);
    hui_ku_update(1);
    assert(hui_ku_stage() == HUI_PROTECT && run.reason == REASON_HAZARD);
    stopped();
    move();
    assert(motor_drive_enabled);

    tick += 20;
    imu.tick = tick - IMU_TIMEOUT_MS - 1;
    move();
    assert(hui_ku_stage() == HUI_PROTECT && run.reason == REASON_IMU);
    stopped();

    tick = captured + HUI_LOSS_MS - 1;
    sample_pose(zone_x, fixture_goal_y, yaw);
    move();
    assert(hui_ku_stage() == HUI_CONFIRM && motor_drive_enabled);
    terminal_assert_no_translation();
    ++tick;
    sample_pose(zone_x, fixture_goal_y, yaw);
    move();
    assert(hui_ku_stage() == HUI_REACQUIRE && run.reason == REASON_LOSS_TIME && run.action == ACTION_STOP);
    stopped();

    tick += 20;
    sample_pose(zone_x, fixture_goal_y, yaw);
    zone_observation(++zone_frame_id, tick, zone_x, fixture_goal_y, yaw, return_zone->kind);
    move();
    assert(hui_ku_stage() == HUI_CONFIRM && motor_drive_enabled);
    key_press(key4_Pin);
    assert(hui_ku_stage() == HUI_STOPPED && !hui_log_active());
    stopped();
    puts("PASS KEY2 terminal protections: 799 ms continues / 800 ms stops to observe, fresh fix restarts, hazard/IMU/KEY4 stop");
}

static void simulated_drive(void)
{
    const float dt = .020f;
    float x = FIELD_START_X_CM, y = FIELD_START_Y_CM, yaw = 0;
    uint32_t start = tick;
    unsigned i;
    memset(&vision, 0, sizeof(vision));
    field_map_reset(tick);
    sample_pose(x, y, yaw);
    key_press(key2_Pin);
    for (i = 0; i < 1500 && hui_ku_stage() != HUI_COMPLETE; ++i) {
        tick += 20;
        sample_pose(x, y, yaw);
        if (i % 4 == 0) {
            const float seen = atan2f(zone_x - x, zone_y - y) - yaw;
            const int in_view = cosf(seen) > .57f;
            zone_observation(++zone_frame_id, tick, x, y, yaw,
                             in_view ? return_zone->kind : VISION_NONE);
        }
        move();
        motor_process();
        {
            const float rotation = dt * body_motion.turn * RETURN_PI / 180.0f;
            const float c = cosf(yaw + rotation / 2), s = sinf(yaw + rotation / 2);
            x += dt * (body_motion.right * c + body_motion.forward * s);
            y += dt * (-body_motion.right * s + body_motion.forward * c);
            yaw += rotation;
        }
    }
    if (hui_ku_stage() != HUI_COMPLETE) {
        fprintf(stderr, "drive stopped with stage=%d x=%.2f y=%.2f yaw=%.2f bearing=%.2f fixes=%lu status=%u\n",
                hui_ku_stage(), x, y, yaw, run.bearing, (unsigned long)run.fixes, run.reason);
    }
    assert(hui_ku_stage() == HUI_COMPLETE);
    assert(fabsf(x - zone_x) <= 2.1f && fabsf(y - fixture_goal_y) <= 2.1f);
    assert(cosf(atan2f(zone_x - x, zone_y - y) - yaw) > cosf(2.01f * RETURN_PI / 180.0f));
    stopped();
    printf("PASS KEY2 idealized drive: start %.1f,%.1f, stop %.2f,%.2f after %.2f s with centre bearing within 2 deg\n",
           FIELD_START_X_CM, FIELD_START_Y_CM, x, y, (double)(tick - start) / 1000.0);
    key_press(key4_Pin);
}

static void rejected_start(void)
{
    task_info.held_kind = VISION_GREEN;
    servo_angle = 120;
    key_press(key2_Pin);
    assert(hui_log_active() && step == 0);
    stopped();
    assert(servo_angle == 120);
    key_press(key4_Pin);
    assert(!hui_log_active());
    collect_session("rejected start", 311032, 1, 0);
}

static void uart_pump(uint32_t *due, uint32_t *bytes)
{
    if (pending && *due == 0) *due = tick + (pending_size * 10U + 114U) / 115U;
    if (pending && tick >= *due) {
        *bytes += pending_size;
        pending = NULL;
        last_complete = tick;
        HAL_UART_TxCpltCallback(&huart5);
        *due = 0;
    }
}

static void uart_feed(const char *text)
{
    while (*text) {
        rx_byte = (uint8_t)*text++;
        faraway_uart_rx_complete(&huart5);
    }
}

static int64_t host_counter(int frequency)
{
#ifdef _WIN32
    LARGE_INTEGER value;
    if (frequency) assert(QueryPerformanceFrequency(&value));
    else assert(QueryPerformanceCounter(&value));
    return value.QuadPart;
#else
    return frequency ? CLOCKS_PER_SEC : clock();
#endif
}

static void minute_run(int diagnostics)
{
    uint32_t flush_deadline;
    const unsigned duration_ms = 60000;
    const float x = zone_x + 12;
    const float y = fixture_goal_y + (zone_y < fixture_goal_y ? 70 : -70);
    const float yaw = atan2f(zone_x - x, zone_y - y);
    const uint32_t begin = tick + 50;
    uint32_t bytes = 0, uart_due = 0, previous_bytes = 0, peak_bps = 0;
    uint32_t tx_start_drops;
    uint32_t samples = 0;
    const double host_us = 1e6 / host_counter(1);
    int64_t control_clocks = 0, control_peak = 0;
    int64_t counts[4] = {0};
    unsigned elapsed;

    reset_wire();
    assert(faraway_init());
    memset(&vision, 0, sizeof(vision));
    tick = begin;
    field_map_reset(tick);
    sample_pose(x, y, yaw);
    if (diagnostics) key_press(key2_Pin);
    tx_start_drops = g_faraway_tx_drop_count;
    /* Test using in-order real UART5 packet generation and 115200/8N1 TX
       completion, with polling intentionally absent from seconds 20-50. */
    for (elapsed = 0; elapsed < duration_ms; ++elapsed, ++tick) {
        int64_t start_clock, control_elapsed;
        uart_pump(&uart_due, &bytes);
        if (diagnostics && (elapsed < 20000U || elapsed >= 50000U)) {
            if (elapsed % 1000U == 0U) uart_feed("KRUN?,419602565\n");
            if (elapsed % 320U == 0U && hui_log.boot) {
                char request[64];
                snprintf(request, sizeof(request), "KREAD,%lu,%lu,%u\n",
                         (unsigned long)hui_log.boot, (unsigned long)hui_log.session, elapsed / 100U);
                uart_feed(request);
            }
        }
        if (elapsed % 20U == 0U) {
            const float dx = 2.0f * sinf((float)elapsed * .005f);
            sample_pose(x + dx, y, yaw);
            counts[0] += 40;
            counts[1] += 42;
            counts[2] -= 36;
            counts[3] -= 44;
            encoder1.count = counts[0]; encoder2.count = counts[1];
            encoder3.count = counts[2]; encoder4.count = counts[3];
            if (elapsed % 100U == 0U)
                zone_observation(++zone_frame_id, tick, x + dx + .4f, y - .7f, yaw, return_zone->kind);
            start_clock = host_counter(0);
            move();
            control_elapsed = host_counter(0) - start_clock;
            control_clocks += control_elapsed;
            if (control_elapsed > control_peak) control_peak = control_elapsed;
            ++samples;
            motor_process();
        }
        faraway_process();
        if (elapsed % 5U == 0) {
            faraway_task_observe();
            faraway_report();
        }
        uart_pump(&uart_due, &bytes);
        if ((elapsed + 1U) % 1000U == 0U) {
            const uint32_t one_second = bytes - previous_bytes;
            if (one_second > peak_bps) peak_bps = one_second;
            previous_bytes = bytes;
        }
    }
    assert(g_faraway_tx_drop_count == tx_start_drops);
    assert(bytes < 1500U * (duration_ms / 1000U));
    printf("PASS KEY2 60s %s: %lu bytes upstream, %.1f B/s, peak fixed second %lu B; fake HAL host move mean %.2f us, max observed %.2f us across %lu cycles\n",
           diagnostics ? "enabled" : "baseline", (unsigned long)bytes, (double)bytes / 60.0,
           (unsigned long)peak_bps, (double)control_clocks * host_us / samples, (double)control_peak * host_us,
           (unsigned long)samples);
    /* Finish the few queued bytes without mutating the clock outside pump. */
    flush_deadline = tick + 2000;
    while (pending || tx_head != tx_tail) {
        assert(tick < flush_deadline);
        ++tick;
        uart_pump(&uart_due, &bytes);
        faraway_tx_start();
    }
    if (diagnostics) {
        key_press(key4_Pin);
        stopped();
        collect_session("60s buffering", 419602566, 1, 0);
    }
}

static void buffer_full(void)
{
    uint8_t payload[84] = {0};
    unsigned i;
    assert(hui_log_begin(tick));
    hui_log_record(1, tick, payload, sizeof(payload));
    for (i = 0; i < 3500; ++i) {
        tick += 20;
        hui_log_record(5, tick, payload, sizeof(payload));
    }
    hui_log_record(7, tick, payload, sizeof(payload));
    hui_log_end();
    collect_session("saturation", 311033, 0, 0);
}

int main(int argc, char **argv)
{
    const int blue = return_zone->kind == VISION_BLUE_MATERIAL_ZONE;
    Motor_Init();
    assert(faraway_init());
    tick = 12000;
    zone_x = blue ? 165 : 135;
    zone_y = blue ? 18 : 282;
    fixture_goal_y = blue ? 68 : 232;
    geometry_and_keys(blue);
    rejected_start();
    if (blue) {
        simulated_drive();
        minute_run(0);
        minute_run(1);
        buffer_full();
    }
    estimated_zone_age_boundary();
    large_zone_fix_field_boundary();
    terminal_yaw_confirmation();
    terminal_yaw_protection();
    if (argc > 1) {
        assert(argc == 2 && blue);
        terminal_log_replay(argv[1]);
    }
    return 0;
}

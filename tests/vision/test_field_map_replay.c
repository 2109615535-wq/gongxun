#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../../../project/Core/Src/field_map.c"

VisionResult vision;
static uint32_t replay_tick;

uint32_t HAL_GetTick(void) { return replay_tick; }

static void require(int condition, const char *path, unsigned line, const char *message)
{
    if (condition) return;
    fprintf(stderr, "%s:%u: %s\n", path, line, message);
    exit(EXIT_FAILURE);
}

static void record_position(uint32_t tick, float x, float y, int first)
{
    if (first) {
        field_map_reset(tick);
        field_map_correct_pose(0, 0, 0);
    }
    field_map_record(tick, x, y, 0);
}

static void replay(const char *path)
{
    unsigned observations[2][FIELD_TRACK_CAPACITY+1U] = {{0}};
    uint32_t first_tick[FIELD_TRACK_CAPACITY+1U] = {0};
    uint32_t *consumed_frames = NULL;
    unsigned frame_capacity = 0;
    unsigned line_number = 0, frames = 0, track_count = 0;
    char line[1024];
    unsigned group, id;
    FILE *input = fopen(path, "r");
    require(input != NULL, path, 0, "could not open replay fixture");
    while (fgets(line, sizeof(line), input)) {
        VisionObservation observation;
        FieldPose observed;
        unsigned fresh, aligned, count, i, first = 0, after = frames;
        unsigned expected_id[sizeof(observation.objects)/sizeof(observation.objects[0])];
        float robot_x, robot_y, yaw_deg;
        const char *cursor = line;
        int consumed = 0;
        memset(&observation, 0, sizeof(observation));
        ++line_number;
        /* One row per status. Target rows have camera XY/distances in mm,
         * and the expected formal ID (0 for an unconfirmed observation). */
        if (line[0]=='S') {
            require(sscanf(cursor, "S %" SCNu32 " %f %f%n", &replay_tick, &robot_x,
                           &robot_y, &consumed)==3 && consumed>0,
                    path, line_number, "invalid state-only status");
            cursor+=consumed;
            require(cursor[strspn(cursor, " \t\r\n")]=='\0', path, line_number, "extra columns");
            record_position(replay_tick, robot_x, robot_y, line_number==1U);
            continue;
        }
        require(sscanf(cursor, "%" SCNu32 " %" SCNu32 " %u %u %f %f %f %f %f %u%n",
                       &replay_tick, &observation.frame_id, &fresh, &aligned,
                       &robot_x, &robot_y, &observed.x, &observed.y, &yaw_deg,
                       &count, &consumed) == 10 && consumed > 0,
                path, line_number, "invalid status header");
        require(fresh <= 1U && aligned <= 1U &&
                count <= sizeof(expected_id)/sizeof(expected_id[0]) &&
                ((fresh && aligned) || !count),
                path, line_number, "invalid flags or target count");
        cursor += consumed;
        observation.updated_ms = replay_tick;
        observation.fresh = (uint8_t)fresh;
        observation.count = (uint8_t)count;
        observation.status = count ? VISION_OBS_OK : VISION_OBS_EMPTY;
        observed.tick = replay_tick; /* The exported pose is already time-aligned. */
        observed.yaw = yaw_deg * DEG_RAD;
        for (i = 0; i < count; ++i) {
            VisionObject *object = &observation.objects[i];
            int32_t x, y, distance;
            consumed = 0;
            require(sscanf(cursor, " %" SCNd32 " %" SCNd32 " %" SCNd32 " %u%n",
                           &x, &y, &distance, &expected_id[i], &consumed) == 4 && consumed > 0 &&
                    distance >= -1 && expected_id[i] <= FIELD_TRACK_CAPACITY,
                    path, line_number, "invalid target");
            cursor += consumed;
            object->kind = VISION_GREEN;
            object->x_mm = x;
            object->y_mm = y;
            object->distance_mm = distance;
            object->position_present = 1;
        }
        require(cursor[strspn(cursor, " \t\r\n")] == '\0', path, line_number, "extra columns");

        record_position(replay_tick, robot_x, robot_y, line_number==1U);
        while (first < after) {
            const unsigned middle = first + (after - first) / 2U;
            if (consumed_frames[middle] < observation.frame_id) first = middle + 1U;
            else after = middle;
        }
        if (first < frames && consumed_frames[first] == observation.frame_id) {
            require(!count, path, line_number, "duplicate frame supplied targets");
            continue;
        }
        if (frames == frame_capacity) {
            uint32_t *expanded;
            frame_capacity = frame_capacity ? frame_capacity * 2U : 128U;
            expanded = realloc(consumed_frames, frame_capacity * sizeof(*consumed_frames));
            require(expanded != NULL, path, line_number, "could not store consumed frame IDs");
            consumed_frames = expanded;
        }
        memmove(consumed_frames + first + 1, consumed_frames + first,
                (frames - first) * sizeof(*consumed_frames));
        consumed_frames[first] = observation.frame_id;
        ++frames;
        apply_frame(&observation, fresh && aligned ? &observed : NULL);

        for (i = 0; i < count; ++i) {
            const FieldPoint *point = &field_map.points[i];
            const VisionObject *object = &observation.objects[i];
            const double yaw = yaw_deg * (3.14159265358979323846 / 180.0);
            const double x = object->x_mm * 0.1, y = object->y_mm * 0.1;
            const double world_x = observed.x + x * cos(yaw) + y * sin(yaw);
            const double world_y = observed.y - x * sin(yaw) + y * cos(yaw);
            require(point->present && fabs(point->x - world_x) < 0.002 &&
                    fabs(point->y - world_y) < 0.002, path, line_number,
                    "raw field point did not match the zero-extrinsic projection");
            id = expected_id[i];
            if (point->track_id != id) {
                fprintf(stderr, "%s:%u: tick=%" PRIu32 " frame=%" PRIu32
                        " target=%u: expected ID %u, got %" PRIu32 "\n",
                        path, line_number, replay_tick, observation.frame_id, i, id, point->track_id);
                exit(EXIT_FAILURE);
            }
            group = range_cm(object) >= 150.0f;
            if (id && !observations[0][id] && !observations[1][id]) {
                first_tick[id] = replay_tick;
                ++track_count;
            }
            ++observations[group][id];
        }
        require(field_map.track_count == track_count, path, line_number,
                "formal track count disagrees with reference IDs");
    }
    require(line_number && !ferror(input), path, line_number, "empty or unreadable fixture");
    fclose(input);
    free(consumed_frames);

    printf("PASS %s: %u status rows, %u distinct upper frames, %u formal IDs\n",
           path, line_number, frames, track_count);
    printf("  pending: near=%u far=%u\n", observations[0][0], observations[1][0]);
    for (id = 1; id <= track_count; ++id) {
        require(field_tracks[id-1].id == id &&
                field_tracks[id-1].observations == observations[0][id] + observations[1][id],
                path, line_number, "formal observation count includes unconfirmed samples");
        printf("  ID %u first_tick=%" PRIu32 " near=%u far=%u\n",
               id, first_tick[id], observations[0][id], observations[1][id]);
    }
}

int main(int argc, char **argv)
{
    if (argc != 3) {
        fprintf(stderr, "usage: %s A.txt B.txt\n", argv[0]);
        return EXIT_FAILURE;
    }
    replay(argv[1]);
    replay(argv[2]);
    return EXIT_SUCCESS;
}

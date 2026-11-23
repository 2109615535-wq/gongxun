/* Actual shared controller and task lifecycle; HAL stubs only. */
#define main existing_firmware_tests
#include "test_static.c"
#undef main

int main(void)
{
    float before;
    assert(pid_steer.kp == 3.5f && pid_steer.ki == 0 && pid_steer.kd == 5);
    assert(TASK_ANGLE_DEAD_DEG == 1);
    Motor_Init(); tick = 1000; task_prepare(); enter_step(30);

    follow_target(30, 1, 25.1f, 50, 1);
    assert(near_align_state == 0 && body_motion.target_forward == 50);
    assert(pid_steer.output == 105); /* Same PD, no first-sample D kick. */
    follow_target(30, 2, 25, 50, 1); motor_process();
    assert(near_align_state == 1 && body_motion.target_forward == 0);
    assert(body_motion.turn == 105 && motor_drive_enabled);
    assert(fabsf(wheel_applied[0] + wheel_applied[1]) < .001f);
    assert(fabsf(wheel_applied[2] + wheel_applied[3]) < .001f);
    before = pid_steer.output;
    follow_target(30, 2, 25, 50, 1);
    assert(pid_steer.output == before); /* Duplicate frame does not update PID. */
    follow_target(20, 3, 27, 50, 1);
    assert(near_align_state == 1 && body_motion.target_forward == 0);
    assert(pid_steer.output == 20); /* 3.5*20 + 5*(20-30). */

    /* Loss clears PID output, but cannot grant another alignment. */
    drive_stop(); stopped(); assert(near_align_state == 1);
    follow_target(10.01f, 4, 20, 50, 1);
    assert(near_align_state == 1 && body_motion.target_forward == 0);
    follow_target(10, 5, 20, 50, 1);
    assert(near_align_state == 2 && body_motion.target_forward == 50);
    follow_target(45, 6, 10, 50, 1);
    assert(near_align_state == 2 && body_motion.target_forward == 50);
    drive_stop(); follow_target(-45, 7, 10, 50, 1);
    assert(near_align_state == 2 && body_motion.target_forward == 50);
    follow_target(1, 8, 10, 50, 1); assert(pid_steer.output == 0);
    follow_target(-1, 9, 10, 50, 1); assert(pid_steer.output == 0);
    follow_target(-1.01f, 10, 10, 50, 1);
    assert(fabsf(pid_steer.output + 3.535f) < .001f);

    task_emergency_stop(); stopped(); assert(near_align_state == 0);
    task_start(); move(); /* Prepare another visual target. */
    follow_target(-10, 11, 24, 50, 1);
    assert(near_align_state == 2 && body_motion.target_forward == 50);
    follow_target(-40, 12, 20, 50, 1);
    assert(near_align_state == 2 && body_motion.target_forward == 50);

    /* The real nearest-target path supplies millimetres converted to cm. */
    task_prepare(); enter_step(30); cameras(260, 30, 0, 0); task_approach();
    assert(near_align_state == 0 && body_motion.target_forward == 50);
    cameras(250, 30, 0, 0); task_approach();
    assert(near_align_state == 1 && body_motion.target_forward == 0);
    tick += 401; task_approach(); stopped(); assert(near_align_state == 1);
    cameras(240, 10, 0, 0); task_approach();
    assert(near_align_state == 2 && body_motion.target_forward == 50);
    cameras(100, 40, 0, 0); task_approach();
    assert(near_align_state == 2 && body_motion.target_forward == 50);
    cameras(100, 40, 0, 1); move();
    assert(step == 40 && servo_angle == 120 && task_info.held_kind == VISION_NONE);
    stopped(); /* ROI still takes priority over alignment and chasing. */

    /* New map point resets the quota; heading loss does not. */
    task_emergency_stop(); field_map_reset(tick);
    assert(task_goto_start(285, FIELD_START_Y_CM + 20, 50));
    field_map.robot.yaw = -.5f; task_goto(285, FIELD_START_Y_CM + 20, 50);
    assert(near_align_state == 1 && body_motion.target_forward == 0);
    field_map.heading_valid = 0; task_goto(285, FIELD_START_Y_CM + 20, 50);
    stopped(); assert(near_align_state == 1);
    field_map.heading_valid = 1; field_map.robot.yaw = 0;
    field_map.robot.tick += 20; task_goto(285, FIELD_START_Y_CM + 20, 50);
    assert(near_align_state == 2 && body_motion.target_forward == 50);
    field_map.robot.yaw = -.5f; field_map.robot.tick += 20;
    task_goto(285, FIELD_START_Y_CM + 20, 50);
    assert(near_align_state == 2 && body_motion.target_forward == 50);
    assert(task_goto_start(285, FIELD_START_Y_CM + 21, 50) && near_align_state == 0);
    task_goto(285, FIELD_START_Y_CM + 21, 50);
    assert(near_align_state == 1 && body_motion.target_forward == 0);
    task_emergency_stop(); stopped();
    assert(pid_steer.kp == 3.5f && pid_steer.ki == 0 && pid_steer.kd == 5);
    puts("PASS: 25cm inclusive/10deg inclusive, pure turn with shared PD, one check per task/point, loss preserves quota, new target resets, +/-1deg deadband, single-frame ROI grip and KEY4 stop");
    return 0;
}

BEGIN { in_b9 = 0; in_b11 = 0; in_b12 = 0 }

/case KART_BENCH_B9_TRACKER:/ {
    print $0
    getline; print $0  # 打印 {
    print "            /* B9: 完整 Tracker 单帧（假设已有点集）。首次需先 Reacquire，"
    print "             * 这里简化：直接调 kart_vtrack_update，若点集为空则测到 0us（骨架验证）。*/"
    print "            (void)kart_vtrack_update((const uint16 *)fake_img_rgb565, 160, 120);"
    in_b9 = 1
    next
}

in_b9 && /break;/ {
    print $0
    in_b9 = 0
    next
}

in_b9 { next }  # 跳过原 TODO 注释

/case KART_BENCH_B11_CONTROL:/ {
    print $0
    getline; print $0  # 打印 {
    print "            /* B11: 速度环 + 转向串级单拍。直接调实时状态，测"控制算法本身"耗时。"
    print "             * 注：实际输入来自编码器/IMU/转向编码器，这里测的是算法而非采集。*/"
    print "            kart_control_speed_update();"
    print "            kart_steer_ctrl_update();"
    in_b11 = 1
    next
}

in_b11 && /break;/ {
    print $0
    in_b11 = 0
    next
}

in_b11 { next }

/case KART_BENCH_B12_END_TO_END:/ {
    print $0
    getline; print $0  # 打印 {
    print "            /* B12: 感知→决策→控制全链路。串联 Tracker + IMU + 控制环。"
    print "             * 当前简化：只测 Tracker + 控制环（IMU 的 Madgwick 在中断里，"
    print "             * 这里单独测会读到过时数据，暂不接入）。*/"
    print "            (void)kart_vtrack_update((const uint16 *)fake_img_rgb565, 160, 120);"
    print "            kart_control_speed_update();"
    print "            kart_steer_ctrl_update();"
    in_b12 = 1
    next
}

in_b12 && /break;/ {
    print $0
    in_b12 = 0
    next
}

in_b12 { next }

{ print }

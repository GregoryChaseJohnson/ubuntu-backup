// dsps_conv_f32_m_ae32.S
.macro conv_f32_ae32 x1, x2, count, step1, step2, name
    lsxp   f0, \x2,  \step2
    loopnez \count, loop_mac_end_\name
        lsxp    f2, \x1, \step1
        madd.s  f1, f2, f0
        lsxp    f0, \x2, \step2
    loop_mac_end_\name:
.endm

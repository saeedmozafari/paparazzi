# Hey Emacs, this is a -*- makefile -*-

# Standard fixed wing control loops


$(TARGET).srcs += $(SRC_FIRMWARE)/stabilization/stabilization_adaptive.c $(SRC_FIRMWARE)/guidance/energy_ctrl_new.c

$(TARGET).CFLAGS += -DCTRL_TYPE_H=\"firmwares/fixedwing/guidance/energy_ctrl_new.h\"
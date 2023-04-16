# Indy7 & RB10 info
Header          header

float32[6]      joint		# [deg]
float32[6]      tcp

float32	        default_speed		# overriding speed [0~1]

                                    # 0: idle 1:moving 2:direct teaching -> INDY
int32	        robot_status		# 1:idle 2:paused/stopped by accident 3:moving -> RB



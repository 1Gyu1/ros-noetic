# Calibration poses (needs replacing with actual values).
Point1 = [679.33, 305.01, 590.07, 59.62, 8.22, 93.90]
Point2 = [679.23, 553.40, 592.39, 59.47, 8.09, 93.79]
Point3 = [688.75, 552.36, 410.20, 59.48, 8.07, 93.76]
Point4 = [688.68, 256.57, 409.27, 59.54, 8.23, 93.81]
Point5 = [688.12, 396.42, 510.19, 52.49, -34.97, 122.86]
Point6 = [688.00, 396.47, 510.23, 34.43, 52.74, 49.86]
Point7 = [688.99, 392.50, 513.73, 58.55, -5.07, 70.22]
Point8 = [689.36, 393.06, 512.32, 62.66, 17.62, 108.60]
Point9 = [689.19, 394.85, 508.60, 74.78, 4.69, 87.64]
Point10 = [687.43, 396.11, 511.34, 37.60, 4.46, 87.80]

# Move to each calibration pose and trigger a calibration plate detection.
movej(Point1)
pickit_find_calibration_plate()

movej(Point2)
pickit_find_calibration_plate()

movej(Point3)
pickit_find_calibration_plate()

movej(Point4)
pickit_find_calibration_plate()

movej(Point5)
pickit_find_calibration_plate()

movej(Point6)
pickit_find_calibration_plate()

movej(Point7)
pickit_find_calibration_plate()

movej(Point8)
pickit_find_calibration_plate()

movej(Point9)
pickit_find_calibration_plate()

movej(Point10)
pickit_find_calibration_plate()
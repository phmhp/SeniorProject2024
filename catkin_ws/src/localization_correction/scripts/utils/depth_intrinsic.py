import pyrealsense2 as rs

# 파이프라인 구성
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)  # Depth 스트림

# 파이프라인 시작
profile = pipeline.start(config)

# 스트림에서 Depth 카메라 프로필 얻기
depth_stream = profile.get_stream(rs.stream.depth)
intrinsics = depth_stream.as_video_stream_profile().get_intrinsics()

# Intrinsic 정보 출력
print(f"Width: {intrinsics.width}")
print(f"Height: {intrinsics.height}")
print(f"Fx: {intrinsics.fx}")
print(f"Fy: {intrinsics.fy}")
print(f"Cx: {intrinsics.ppx}")
print(f"Cy: {intrinsics.ppy}")
print(f"Distortion Model: {intrinsics.model}")
print(f"Distortion Coefficients: {intrinsics.coeffs}")

pipeline.stop()


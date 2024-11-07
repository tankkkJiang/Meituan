# 目录
cd Meituan
# 拉取代码- GitHub账号配置好的 重新取名 教程
git fetch origin
git reset --hard origin/main
# 授予权限
chmod +x stop_race.sh
chmod +x start_race.sh
chmod +x copy_runfiles.sh
# 原来的容器关掉
./stop_race.sh
# 重新启动容器
./start_race.sh
# 拷贝文件
./copy_runfiles.sh
# 进入到容器当中
docker exec -it race_user_sdk_container /bin/bash

cd home
./first_run_demo.sh
exit


docker cp race_demo/src/scripts/ race_user_sdk_container:/root/catkin_ws/src/race_demo/src/
docker exec -it race_user_sdk_container /bin/bash 

chmod +x /root/catkin_ws/src/race_demo/src/scripts/cs.py
cp /home/sdk_for_user/map_client_sdk/for_py/* /root/catkin_ws/src/race_demo/src/scripts/
cp /home/sdk_for_user/msg/* /root/catkin_ws/src/race_demo/msg/
cp /home/sdk_for_user/map_client_sdk/for_py/voxel_map_final.bin /root/catkin_ws/
./home/run.sh

# 获取环境的日志
docker exec -it race_scene_sdk_container /bin/bash
docker cp race_scene_sdk_container:/evaluator/config/race.log .


# 提交
appKey：caef89855ac242dde4285a160cebd253
appSecret：d97f48248c7eb859fa612eed9451adda


docker commit race_user_sdk_container  race_user:1006.888

docker tag race_user:1006.888 uav-challenge.tencentcloudcr.com/uav_challenge_2024/caef89855ac242dde4285a160cebd253:1006.888

docker push uav-challenge.tencentcloudcr.com/uav_challenge_2024/caef89855ac242dde4285a160cebd253:1006.888
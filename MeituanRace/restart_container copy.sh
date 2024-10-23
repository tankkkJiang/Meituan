# 目录
cd MeituanRace/Meituan/MeituanRace
# 拉取代码- GitHub账号配置好的 重新取名 教程
git fetch origin
git reset --hard origin/main
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

# 修改文件权限
chmod +x /root/catkin_ws/src/race_demo/src/scripts/cs.py
cp /home/sdk_for_user/map_client_sdk/for_py/* /root/catkin_ws/src/race_demo/src/scripts/
cp /home/sdk_for_user/msg/* /root/catkin_ws/src/race_demo/msg/
cp /home/sdk_for_user/map_client_sdk/for_py/voxel_map_final.bin /root/catkin_ws/
./home/run.sh



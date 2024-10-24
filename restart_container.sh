cd Meituan
git fetch origin
git reset --hard origin/main
chmod +x stop_race.sh
chmod +x start_race.sh
chmod +x copy_runfiles.sh
./stop_race.sh
./start_race.sh
./copy_runfiles.sh
docker exec -it race_user_sdk_container /bin/bash


cd home
./first_run_demo.sh
exit


docker cp race_demo/src/scripts/ race_user_sdk_container:/root/catkin_ws/src/race_demo/src/
docker exec -it race_user_sdk_container /bin/bash 

chmod +x /root/catkin_ws/src/race_demo/src/scripts/tank.py
cp /home/sdk_for_user/map_client_sdk/for_py/* /root/catkin_ws/src/race_demo/src/scripts/
cp /home/sdk_for_user/msg/* /root/catkin_ws/src/race_demo/msg/
cp /home/sdk_for_user/map_client_sdk/for_py/voxel_map_final.bin /root/catkin_ws/
./home/run.sh



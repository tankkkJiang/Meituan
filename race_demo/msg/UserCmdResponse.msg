# CmdResponseType
uint8 type
uint8 CMD_RESP_SUCCESS = 0
uint8 ERROR_TYPE_DRONE_NOT_READY = 1 # 飞机不是ready状态
uint8 ERROR_TYPE_DRONE_PLAN_ROUTE_VERIFICATION_FAILED = 2 # 飞机航线校验失败
uint8 ERROR_TYPE_CAR_CANNOT_EXECUTE_RELEASE_OPERATION = 3 # 飞机无法执行释放动作
uint8 ERROR_TYPE_DRONE_NOT_ON_AVIATION_OPERATION_POINT = 4  # 飞机不在航空作业点
uint8 ERROR_TYPE_DRONE_NOT_ON_BIRTHPLACE = 5               # 飞机不在出生地
uint8 ERROR_TYPE_DRONE_NOT_ON_LOADING_CARGO_STATION = 6   # 飞机不在上货点
uint8 ERROR_TYPE_DRONE_NOT_ON_UNLOADING_CARGO_STATION = 7  # 飞机不在卸货站
uint8 ERROR_TYPE_DRONE_BINDING_OTHER_CARGO = 8             # 飞机绑定了其他货物
uint8 ERROR_TYPE_DRONE_BINDING_OTHER_CAR = 9               # 飞机绑定了其他小车
uint8 ERROR_TYPE_DRONE_NO_BINDING_CARGO = 10               # 飞机未绑定货物
uint8 ERROR_TYPE_DRONE_CANNOT_EXECUTE_BATTERY_REPLACEMENT = 11 # 飞机无法执行换电操作
uint8 ERROR_TYPE_CAR_NOT_READY = 12    # 小车不是ready状态
uint8 ERROR_TYPE_CAR_PLAN_ROUTE_VERIFICATION_FAILED = 13  # 小车规划路径校验失败
uint8 ERROR_TYPE_CAR_HAVE_OTHER_DRONE = 14                # 小车上有其他飞机
uint8 ERROR_TYPE_CAR_NOT_ON_LOADING_CARGO_STATION = 15        # 小车不在上货点
uint8 ERROR_TYPE_NO_BINDING_CAR_AND_DRONE = 16             # 飞机和小车不是绑定关系
uint8 ERROR_TYPE_CARGO_NO_NOT_STARTED = 17                # 货物不是NOT_STARTED状态
uint8 ERROR_TYPE_CARGO_NO_DELIVERY = 18                   # 货物不是配送状态
uint8 ERROR_TYPE_CARGO_WRONG_DESTINATION = 19             # 货物被送至错误的目的地
uint8 ERROR_TYPE_LOADING_CARGO_STATION_NO_DRONE = 20      # 上货点没有飞机
uint8 ERROR_TYPE_LOADING_CARGO_STATION_NO_CAR = 21        # 上货点没有小车
uint8 ERROR_TYPE_DRONE_IN_FLYING_RELEASE_CARGO = 22       # 飞机在飞行中释放了货物

string description
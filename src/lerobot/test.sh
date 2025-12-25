# python xlerobot_manual_joint_control.py --port /dev/ttyACM1  --raw --joint left_arm_gripper --value 3500 --speed slow
# python xlerobot_manual_joint_control.py --speed slow --port /dev/ttyACM1

# python xlerobot_manual_joint_control.py --port /dev/ttyACM1  --raw --interactive 

# python xlerobot_manual_joint_control.py --port /dev/ttyACM1   --joint left_arm_gripper --value 0
#   left_arm_shoulder_pan          - Shoulder pan rotation (horizontal rotation)
#   left_arm_shoulder_lift         - Shoulder lift (up/down)
#   left_arm_elbow_flex            - Elbow flex (bend)
#   left_arm_wrist_flex            - Wrist flex (up/down)
#   left_arm_wrist_roll            - Wrist roll (rotation)
#   left_arm_gripper 
#   --port PORT      USB port for the robot (default: /dev/ttyACM0)
#   --use-right-arm  Use right arm instead of left arm
#   --raw            Use raw motor values (0-4095) instead of normalized values
#   --joint JOINT    Joint name to control (e.g., left_arm_shoulder_pan)
#   --value VALUE    Value to set (angle/velocity for normal mode, raw integer for raw mode)
#   --interactive    Enter interactive mode for multiple commands
#   --list           List all available joints
# python -m lerobot.robots.xlerobot.xlerobot_single_arm_host --robot.id=None
# python -m lerobot.4_xlerobot_single_arm_teleop_leader_client
python -m lerobot.record \
    --robot.type=xlerobot_single_arm_client \
    --robot.remote_ip=192.168.110.29 \
    --robot.id=my_xlerobot \
    --teleop.type=so101_leader \
    --teleop.port=COM3 \
    --teleop.id=my_leader_arm \
    --dataset.repo_id=your_username/xlerobot_dataset \
    --dataset.single_task="把橘子放到篮子里" \
    --dataset.num_episodes=10 \
    --display_data=true

python -m lerobot.record `
    --robot.type=xlerobot_single_arm_client `
    --robot.remote_ip=192.168.110.29 `
    --robot.id=my_xlerobot `
    --teleop.type=so101_leader `
    --teleop.port=COM3 `
    --teleop.id=my_leader_arm `
    --dataset.repo_id=wangranryan/xlerobot_lsd `
    --dataset.single_task="把螺丝刀放到篮子里" `
    --dataset.num_episodes=5 `
    --display_data=true `
    --play_sounds=false `
    --resume=true 

python -m lerobot.record `
    --robot.type=xlerobot_single_arm_client `
    --robot.remote_ip=192.168.110.29 `
    --robot.id=my_xlerobot `
    --teleop.type=so101_leader `
    --teleop.port=COM3 `
    --teleop.id=my_leader_arm `
    --dataset.repo_id=wangranryan/xlerobot_lsd `
    --dataset.single_task="把螺丝刀放到篮子里" `
    --dataset.num_episodes=10 `
    --display_data=true `
    --play_sounds=false 

huggingface-cli upload wangranryan/xlerobot_lsd "C:\Users\wr\.cache\huggingface\lerobot\wangranryan\xlerobot_lsd" --repo-type dataset

lerobot-train `
  --policy.path=lerobot/smolvla_base `
  --policy.repo_id=wangranryan/xlerobot_lsd_2 `
  --dataset.repo_id=wangranryan/xlerobot_lsd `
  --batch_size=16 `
  --steps=20000 `
  --output_dir=outputs/train/my_smolvla_bs16 `
  --job_name=my_smolvla_training `
  --policy.device=cuda `
  --wandb.enable=false


python src/lerobot/scripts/server/policy_server.py --host=0.0.0.0 --port=8080

# 启动robot client (在另一个终端)
python src/lerobot/scripts/server/robot_client.py \
    --robot.type=xlerobot_single_arm \
    --robot.port1=COM3 \
    --robot.cameras="{ left_wrist: {type: opencv, index_or_path: /dev/video0, width: 640, height: 480, fps: 30}}" \
    --task="dummy" \
    --server_address=127.0.0.1:8080 \
    --policy_type=act \
    --pretrained_name_or_path=user/model \
    --policy_device=cuda \
    --actions_per_chunk=50 \
    --chunk_size_threshold=0.5

    





python -m src.lerobot.scripts.server.robot_client \
    --server_address=192.168.110.39:8080 \ # SERVER: the host address and port of the policy server
    --robot.type=xlerobot_single_arm \ # ROBOT: your robot type
    --robot.port=/dev/ttyACM0 \ # ROBOT: your robot port
    --robot.id=None \ # ROBOT: your robot id, to load calibration file
    --robot.cameras="{ left_wrist: {type: opencv, index_or_path: 2, width: 640, height: 480, fps: 30}, head(RGDB): {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}}" \ # POLICY: the cameras used to acquire frames, with keys matching the keys expected by the policy
    --task="把螺丝刀放到篮子里" \ # POLICY: The task to run the policy on (`Fold my t-shirt`). Not necessarily defined for all policies, such as `act`
    --policy_type=smolvla \ # POLICY: the type of policy to run (smolvla, act, etc)
    --pretrained_name_or_path=wangranryan/xlerobot_lsd_2 \ # POLICY: the model name/path on server to the checkpoint to run (e.g., lerobot/smolvla_base)
    --policy_device=cuda \ # POLICY: the device to run the policy on, on the server
    --actions_per_chunk=50 \ # POLICY: the number of actions to output at once
    --chunk_size_threshold=0.5 \ # CLIENT: the threshold for the chunk size before sending a new observation to the server
    --aggregate_fn_name=weighted_average \ # CLIENT: the function to aggregate actions on overlapping portions
    --debug_visualize_queue_size=True # CLIENT: whether to visualize the queue size at runtime
    

python -m src.lerobot.scripts.server.robot_client \
    --server_address=192.168.110.39:8080 \
    --robot.type=xlerobot_single_arm \
    --robot.port1=/dev/ttyACM0 \
    --robot.id=None \
    --robot.cameras='{"left_wrist": {"type": "opencv", "index_or_path": 2, "width": 640, "height": 480, "fps": 30}, "head": {"type": "opencv", "index_or_path": 0, "width": 640, "height": 480, "fps": 30}}' \
    --task="把螺丝刀放到篮子里" \
    --policy_type=smolvla \
    --pretrained_name_or_path=wangranryan/xlerobot_lsd_2 \
    --policy_device=cuda \
    --actions_per_chunk=50 \
    --chunk_size_threshold=0.5 \
    --aggregate_fn_name=weighted_average \
    --debug_visualize_queue_size=True

    
git fetch origin && git reset --hard origin/main
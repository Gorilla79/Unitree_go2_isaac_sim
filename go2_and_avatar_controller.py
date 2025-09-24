from isaacsim import SimulationApp

simulation_app = SimulationApp({"renderer": "RayTracedLighting", "headless": False})

import sys
import carb
import omni
from omni.isaac.core import World
from omni.isaac.core.utils.extensions import enable_extension
from omni.isaac.nucleus import get_assets_root_path
from pxr import UsdGeom, Sdf, Gf, Usd
import omni.graph.core as og
import omni.kit.commands
import omni.isaac.core.utils.prims as prims_utils

import time
import numpy as np
enable_extension("isaacsim.ros2.bridge")

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, String

enable_extension("omni.anim.retarget.bundle")
simulation_app.update()

class Go2AndAvatarController(Node):
    def __init__(self):
        super().__init__("go2_and_avatar_controller")
        self.usd_context = omni.usd.get_context()
        self.stage = self.usd_context.get_stage()

        # 4floor 환경 프림 및 경로 설정
        self.env_prim_path = "/World/Dongeui4floor"
        self.env_usd_path = "omniverse://localhost/Users/gorilla79/Dongeui4floor/4floor.usd"
        print("환경(Stage) 경로:", self.env_usd_path)

        # 환경 프림 정의 및 참조 추가
        env_prim = self.stage.DefinePrim(self.env_prim_path, "Xform")
        env_prim.GetReferences().AddReference(self.env_usd_path)
        time.sleep(1)  # 환경 로딩 대기
        
        # Go2 관련 경로 및 변수 초기화
        self.go2_root_path = "/go2_description"
        self.go2_articulation_path = f"{self.go2_root_path}/base"
        self.assets_root_path = get_assets_root_path()
        if self.assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            simulation_app.close()
            sys.exit()
        
        self.avatars = {}
        #self.simple_avatar_asset_path = self.assets_root_path + "/Isaac/People/Characters/original_male_adult_construction_03/male_adult_construction_03.usd"
        self.simple_avatar_asset_path = "omniverse://localhost/Users/gorilla79/retargetmen.usd"
        print("아바타(Stage) 경로:", self.simple_avatar_asset_path)

        self.timeline = omni.timeline.get_timeline_interface()
        self.world = World(stage_units_in_meters=1.0)
        self.world.scene.add_default_ground_plane()

        # Go2 로봇 및 OmniGraph 셋업
        self.setup_go2_robot()
        self.setup_go2_omnigraph()

        # ROS2 구독자 등록 및 기타 초기화...
        self.detection_sub = self.create_subscription(Int32MultiArray, "person_detections", self.detection_callback, 10)

        self.world.reset()
        print("초기화 완료. /person_detections 토픽 구독 중...")


    def setup_go2_robot(self):
        go2_asset_path = self.assets_root_path + "/Isaac/Robots/Unitree/Go2/go2.usd"
        omni.kit.commands.execute('DeletePrims', paths=[self.go2_root_path])
        stage = self.stage
        go2_prim = stage.DefinePrim(self.go2_root_path, "Xform")
        go2_prim.GetReferences().AddReference(go2_asset_path)
        time.sleep(1)
        go2_prim = stage.GetPrimAtPath(self.go2_root_path)
        if not go2_prim.IsValid():
            carb.log_error(f"[ERROR] Go2 prim not found at {self.go2_root_path}")
            simulation_app.close()
            sys.exit()
            
        omni.kit.commands.execute(
            "TransformPrimSRT",
            path=Sdf.Path(self.go2_root_path),
            new_translation=Gf.Vec3f(0.0, 0.0, 0.5),  # Z축 15
            new_rotation_euler=Gf.Vec3f(0, 0, 0),
            new_rotation_order=Gf.Vec3i(0, 1, 2),
            new_scale=Gf.Vec3f(1, 1, 1),
        )
        
        self.stage = stage
        

    def setup_go2_omnigraph(self):
        articulation_prim = self.stage.GetPrimAtPath(self.go2_articulation_path)
        if not articulation_prim.IsValid():
            carb.log_error(f"[ERROR] Go2 articulation prim not found at {self.go2_articulation_path}")
            return
        og.Controller.edit(
            {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
                    ("SubscribeJointState", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                    ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
                    ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "SubscribeJointState.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
                    ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
                    ("SubscribeJointState.outputs:jointNames", "ArticulationController.inputs:jointNames"),
                    ("SubscribeJointState.outputs:positionCommand", "ArticulationController.inputs:positionCommand"),
                    ("SubscribeJointState.outputs:velocityCommand", "ArticulationController.inputs:velocityCommand"),
                    ("SubscribeJointState.outputs:effortCommand", "ArticulationController.inputs:effortCommand"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("ArticulationController.inputs:robotPath", self.go2_articulation_path),
                    ("PublishJointState.inputs:targetPrim", self.go2_articulation_path),
                ],
            },
        )
        print("[INFO] OmniGraph Action Graph가 Python에서 자동으로 생성/설정되었습니다.")

    def get_go2_base_world_position(self):
        xform_cache = UsdGeom.XformCache(Usd.TimeCode.Default())
        go2_base_prim = self.stage.GetPrimAtPath(self.go2_articulation_path)
        if not go2_base_prim.IsValid():
            print(f"Go2 base prim not found: {self.go2_articulation_path}")
            return Gf.Vec3d(0, 0, 0)
        world_xform = xform_cache.GetLocalToWorldTransform(go2_base_prim)
        return world_xform.ExtractTranslation()
    
    def create_avatar(self, avatar_id):
        avatar_path = f"/World/Avatar_{avatar_id}"
        if self.stage.GetPrimAtPath(Sdf.Path(avatar_path)).IsValid():
            prims_utils.delete_prim(avatar_path)
        omni.kit.commands.execute('CreatePayloadCommand',
            usd_context=self.usd_context,
            path_to=Sdf.Path(avatar_path),
            asset_path=self.simple_avatar_asset_path,
            instanceable=False)
        time.sleep(0.3)
        self.avatars[avatar_id] = avatar_path  # 생성 후 위치 설정은 호출자에서 처리

    def update_avatar_position(self, avatar_id, relative_position: Gf.Vec3d):
        if avatar_id not in self.avatars:
            self.create_avatar(avatar_id)
            # 위치 업데이트는 호출자에서 따로 해줌
            return False  # 위치 미처리 상태 표시

        avatar_path = self.avatars[avatar_id]
        go2_world_pos = self.get_go2_base_world_position()
        absolute_pos = go2_world_pos + relative_position

         # z 값을 0으로 고정
        absolute_pos = Gf.Vec3d(absolute_pos[0], absolute_pos[1], 0.0)

        omni.kit.commands.execute('TransformPrimSRT',
            path=Sdf.Path(avatar_path),
            new_translation=absolute_pos,
            new_rotation_euler=Gf.Vec3f(0, 0, -90),
            new_rotation_order=Gf.Vec3i(0, 1, 2),
            new_scale=Gf.Vec3f(1, 1, 1)
        )
        print(f"아바타 위치 갱신: ID={avatar_id}, 절대 위치={absolute_pos}")
        return True

    def detection_callback(self, msg: Int32MultiArray):
        if not self.world.is_playing():
            return
        data = msg.data
        detected_ids = set()
        for i in range(0, len(data), 3):
            if i + 2 >= len(data):
                continue
            avatar_id = data[i]
            x_m = data[i+1] / 1000.0  # 실제 로봇 기준 x (미터)
            y_m = data[i+2] / 1000.0  # 실제 로봇 기준 y (미터)
            
            # 실제 +y 방향을 가상 -y 방향으로 변환
            x_sim = y_m
            y_sim = -x_m
            
            pos = Gf.Vec3d(x_sim, y_sim, 0)
            
            detected_ids.add(avatar_id)
            updated = self.update_avatar_position(avatar_id, pos)
            if not updated:
                # 방금 생성했으면 위치도 바로 설정해줌
                self.update_avatar_position(avatar_id, pos)
        
        # 삭제 대상 아바타 처리
        to_remove = [aid for aid in self.avatars if aid not in detected_ids]
        for rid in to_remove:
            self.remove_avatar(rid)


    def remove_avatar(self, avatar_id):
        if avatar_id not in self.avatars:
            return
        prim_path = self.avatars[avatar_id]
        if prims_utils.is_prim_path_valid(prim_path):
            prims_utils.delete_prim(prim_path)
        del self.avatars[avatar_id]
        print(f"아바타 삭제: ID={avatar_id}")

    def run_simulation(self):
        self.timeline.play()
        reset_needed = False
        while simulation_app.is_running():
            self.world.step(render=True)
            rclpy.spin_once(self, timeout_sec=0.001)
            if self.world.is_playing() and self.world.is_stopped():
                reset_needed = True
            if self.world.is_playing():
                if reset_needed:
                    print("시뮬레이션 리셋 감지, 아바타 상태 초기화")
                    self.world.reset()
                    for aid in list(self.avatars.keys()):
                        self.remove_avatar(aid)
                    reset_needed = False
        self.timeline.stop()
        self.destroy_node()
        simulation_app.close()

if __name__ == "__main__":
    rclpy.init()
    controller = Go2AndAvatarController()
    try:
        controller.run_simulation()
    except Exception as e:
        print(f"시뮬레이션 오류: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        print("애플리케이션 종료.")


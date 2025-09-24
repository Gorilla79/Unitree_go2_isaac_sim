# Unitree_go2_isaac_sim

'go2_grand_ICT_v5.py'실행 시 /persond_detections와 /danger 토픽 발행
- 해당 토픽은 사람의 위치 좌표와 위험 여부에 대한 색상 정보
- 'go2_grand_ICT_v4.cpp'가 실제로 실행됨
  
'Go2JointStateBridge.py'는 12개의 조인트 값을 받아서 /joint_command 토픽으로 발행 
- 'go2_joint_state_wrapper.cpp'가 실제 동작함
  
### [개선점]
- 이미지 데이터와 좌표 값+색상 정보를 전달하다 보니 데이터가 느리게 전송받는 것을 확인할 수 있음
- 이미지 데이터를 없애는 것을 추천
- 이미지 분석 결과는 API나 클라우드를 통해 확인하는 것으로 방법을 모색하는 것을 추천
- 이미지 데이터를 삭제하고 JointState 값을 동시에 보내서 동기화를 하는 방안으로 !!
- 'go2_grand_ICT_v5.py'는 사람 추적 및 방어에 대한 기능이 실행안됨(삭제되었음) -> 'go2_grand_ICT_v4.py'를 실행 or 기능이 포함된 'go2_grand_ICT_v6.py'(예정)를 실행

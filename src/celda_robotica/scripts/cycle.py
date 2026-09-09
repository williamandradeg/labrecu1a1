#!/usr/bin/env python3
from pathlib import Path
import time,json,csv,math,threading
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node as RosNode
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile,DurabilityPolicy
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import String
from controller_manager_msgs.srv import ListControllers
from gz.transport13 import Node
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.empty_pb2 import Empty
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.twist_pb2 import Twist
from gz.msgs10.stringmsg_pb2 import StringMsg

ROOT=Path.home()/'lrrecupera'
J=['shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint']

class Cycle(RosNode):
    def __init__(self):
        super().__init__('coordinador_celda',parameter_overrides=[rclpy.parameter.Parameter('use_sim_time',value=True)])
        self.gz=Node();self.poses={};self.sim=0.;self.attached={};self.state='INICIALIZANDO';self.lastrow=-1
        self.gz.subscribe(Pose_V,'/celda/poses',self.pose_cb)
        self.pubs={}
        for robot in ['ur3','ur5','mir']:
            for cmd in ['attach','detach']:self.pubs[robot+'/'+cmd]=self.gz.advertise('/celda/'+robot+'/'+cmd,Empty)
            self.gz.subscribe(StringMsg,'/celda/'+robot+'/attached',lambda msg,r=robot:self.attached.update({r:msg.data}))
        self.belt=self.gz.advertise('/celda/belt/run',Boolean)
        self.vel=self.gz.advertise('/celda/mir/cmd_vel',Twist)
        self.action_clients={r:ActionClient(self,FollowJointTrajectory,f'/{r}/joint_trajectory_controller/follow_joint_trajectory') for r in ['ur3','ur5']}
        self.waypoints=json.loads((ROOT/'src/celda_robotica/config/waypoints.json').read_text())
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status=self.create_publisher(String,'/celda/estado',qos)
        self.events=[]
        self.csvfile=(ROOT/'evidencias/telemetria.csv').open('w',newline='')
        self.writer=csv.writer(self.csvfile);self.writer.writerow(['t_sim_s','estado','box_x','box_y','box_z','mir_x','mir_y','mir_z'])

    def pose_cb(self,msg):
        self.sim=msg.header.stamp.sec+msg.header.stamp.nsec*1e-9
        for p in msg.pose:
            self.poses[p.name]=([p.position.x,p.position.y,p.position.z],[p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w])
        if hasattr(self,'writer') and not self.csvfile.closed and self.sim-self.lastrow>.1 and 'caja' in self.poses and 'mir100' in self.poses:
            self.writer.writerow([self.sim,self.state,*self.poses['caja'][0],*self.poses['mir100'][0]]);self.lastrow=self.sim

    def phase(self,s):
        self.state=s;self.get_logger().info(s)
        self.status.publish(String(data=s))
        self.events.append({'t_sim_s':self.sim,'fase':s,'caja':self.poses.get('caja'),'mir':self.poses.get('mir100')})
        (ROOT/'evidencias/eventos.json').write_text(json.dumps(self.events,indent=2))
        (ROOT/'evidencias/estado.txt').write_text(s)
        self.csvfile.flush()

    def wait(self,seconds):
        t=self.sim;deadline=time.monotonic()+max(60,seconds*20)
        while self.sim-t<seconds:
            if time.monotonic()>deadline:raise RuntimeError('La simulacion esta pausada o no responde')
            rclpy.spin_once(self,timeout_sec=.02)

    def until(self,predicate,sim_timeout=40):
        t=self.sim;deadline=time.monotonic()+max(90,sim_timeout*20)
        while not predicate():
            if self.sim-t>sim_timeout or time.monotonic()>deadline:raise RuntimeError('Tiempo agotado esperando condicion de proceso')
            rclpy.spin_once(self,timeout_sec=.02)

    def move(self,robot,target,duration=3.):
        c=self.action_clients[robot]
        if not c.wait_for_server(timeout_sec=90):raise RuntimeError('Controlador no disponible: '+robot)
        goal=FollowJointTrajectory.Goal()
        goal.trajectory.joint_names=[robot+'_'+j for j in J]
        point=JointTrajectoryPoint(positions=self.waypoints[robot][target]['q'],velocities=[0.]*6)
        point.time_from_start.sec=int(duration);point.time_from_start.nanosec=int((duration-int(duration))*1e9)
        goal.trajectory.points=[point]
        f=c.send_goal_async(goal);rclpy.spin_until_future_complete(self,f,timeout_sec=60)
        if not f.done() or not f.result().accepted:raise RuntimeError('Trayectoria rechazada '+robot+'/'+target)
        res=f.result().get_result_async();rclpy.spin_until_future_complete(self,res,timeout_sec=180)
        if not res.done() or res.result().result.error_code!=0:raise RuntimeError('Trayectoria fallida '+robot+'/'+target+': '+str(res.result()))
        self.wait(.25)

    def attach(self,robot):
        if robot!='mir':
            pos,quat=self.poses[robot+'_tool']
            tip=np.array(pos)+Rotation.from_quat(quat).apply([0,0,.11])
            box=np.array(self.poses['caja'][0])+[0,0,.08]
            err=np.linalg.norm(tip-box)
            if err>.025:raise RuntimeError(f'Ventosa fuera de contacto {robot}: {err:.4f} m tip={tip} box={box}')
        self.pubs[robot+'/attach'].publish(Empty())
        self.until(lambda:self.attached.get(robot)=='attached',5)
        self.wait(.4)

    def detach(self,robot):
        self.pubs[robot+'/detach'].publish(Empty())
        self.until(lambda:self.attached.get(robot)=='detached',5)
        self.wait(.4)

    def ready(self):
        for robot in ['ur3','ur5']:
            client=self.create_client(ListControllers,f'/{robot}/controller_manager/list_controllers')
            deadline=time.monotonic()+150
            active=False
            while time.monotonic()<deadline:
                if client.wait_for_service(timeout_sec=2):
                    future=client.call_async(ListControllers.Request())
                    rclpy.spin_until_future_complete(self,future,timeout_sec=5)
                    if future.done() and future.result():
                        active=any(c.name=='joint_trajectory_controller' and c.state=='active' for c in future.result().controller)
                        if active:break
                time.sleep(.5)
            self.destroy_client(client)
            if not active:raise RuntimeError('Controlador no activo: '+robot)

    # Aqui controlamos toda la secuencia de la simulación
    # Los tiempos fueron ajustadospara que fuese fluida en sus movimientos
    # En la consola se puede ir viendo en ue paso va la simulación
    # La ejecución es exitosa si cargamos primero la parte de gazebo y esperamos a que se dibuje bien la celda 
    # y ejecutamos el script en otra terminal, no en la que iniciamos el proceso
    def run(self):
        self.ready()
        self.until(lambda:all(k in self.poses for k in ['caja','mir100','ur3_tool','ur5_tool']),30)
        self.belt.publish(Boolean(data=False));self.vel.publish(Twist())
        self.wait(1)
        self.phase('01 LISTA Caja sobre mesa de entrada')
        self.wait(2)
        self.move('ur3','pick_above',3)
        self.phase('02 UR3 Aproximacion vertical')
        self.move('ur3','pick',2)
        self.attach('ur3')
        self.phase('03 UR3 Caja sujeta por ventosa')
        self.move('ur3','lift',2.5)
        self.wait(1.5)
        self.phase('04 UR3 Traslado al transportador')
        self.move('ur3','place_above',4)
        self.move('ur3','place',2)
        self.detach('ur3')
        self.phase('05 Caja depositada sobre banda')
        self.move('ur3','retreat',2)
        self.move('ur3','home',2)
        self.phase('06 Banda transportando caja')
        self.belt.publish(Boolean(data=True))
        self.until(lambda:self.poses['caja'][0][0]>=1.13,35)
        self.belt.publish(Boolean(data=False))
        self.phase('07 Sensor final Caja en estacion UR5')
        self.move('ur5','pick_above',3)
        self.move('ur5','pick',2)
        self.attach('ur5')
        self.phase('08 UR5 Caja sujeta por ventosa')
        self.move('ur5','lift',2.5)
        self.wait(1.5)
        self.move('ur5','place_above',4)
        self.phase('09 UR5 Deposito sobre MiR100')
        self.move('ur5','place',3)
        self.detach('ur5')
        self.attach('mir')
        self.phase('10 MiR100 cargado')
        self.move('ur5','retreat',3)
        self.move('ur5','home',3)
        self.phase('11 MiR100 Trayecto recto a mesa final')
        begin=self.sim
        while self.poses['mir100'][0][0]<3.57:
            if self.sim-begin>30:raise RuntimeError('MiR no llega a destino')
            msg=Twist();msg.linear.x=min(.20,max(.025,(3.6-self.poses['mir100'][0][0])*.7));self.vel.publish(msg)
            self.wait(.10)
        self.vel.publish(Twist());self.wait(2)
        box=self.poses['caja'][0];mir=self.poses['mir100'][0]
        if math.dist(box[:2],mir[:2])>.1 or not .60<box[2]<.72:raise RuntimeError('Caja fuera de plataforma al finalizar')
        if abs(mir[1]+.53)>.08:raise RuntimeError('Desviacion lateral excesiva')
        self.phase('12 CICLO COMPLETADO MiR detenido frente a mesa')
        (ROOT/'evidencias/resultado.json').write_text(json.dumps({'resultado':'PASS','t_fin_sim_s':self.sim,'box':box,'mir':mir,'events':len(self.events)},indent=2))
        self.csvfile.flush()
        
def main():
    rclpy.init();node=Cycle()
    try:node.run()
    except Exception as ex:
        node.belt.publish(Boolean(data=False));node.vel.publish(Twist())
        node.phase('ERROR '+str(ex))
        (ROOT/'evidencias/resultado.json').write_text(json.dumps({'resultado':'FAIL','error':str(ex)},indent=2))
        raise
    finally:
        node.gz.unsubscribe('/celda/poses');node.csvfile.close();node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()

#Aquí generamos la escena completa
# Los brazos robóticos y el mir han sido tomados de sitios oficiales
# para que así se vean más reales en la simulación

from pathlib import Path
import subprocess, xml.etree.ElementTree as ET, math, json, shutil
import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.optimize import least_squares

ROOT=Path(__file__).resolve().parents[3]
P=ROOT/'src/celda_robotica'
J=['shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint']
BASE={'ur3':[-1.25,-.38,.80],'ur5':[1.30,-.53,.80]}
CONFIG={}

def write(p,s): p.write_text(s)

def ur(typ,initial=None):
    pre=typ+'_'
    cfg='/opt/ros/jazzy/share/ur_description/config/'+typ
    init='' if initial is None else ' initial_positions="${'+'dict('+','.join(j+'='+str(v) for j,v in zip(J,initial))+')'+'}"'
    text=f'''<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="{typ}">
<xacro:include filename="/opt/ros/jazzy/share/ur_description/urdf/ur_macro.xacro"/>
<xacro:include filename="/opt/ros/jazzy/share/ur_simulation_gz/urdf/ur_gz.ros2_control.xacro"/>
<link name="world"/>
<xacro:ur_robot name="{typ}" tf_prefix="{pre}" parent="world" joint_limits_parameters_file="{cfg}/joint_limits.yaml" kinematics_parameters_file="{cfg}/default_kinematics.yaml" physical_parameters_file="{cfg}/physical_parameters.yaml" visual_parameters_file="{cfg}/visual_parameters.yaml" force_abs_paths="true">
<origin xyz="{' '.join(map(str,BASE[typ]))}" rpy="0 0 0"/>
</xacro:ur_robot>
<link name="{pre}vacuum">
<inertial><origin xyz="0 0 .05"/><mass value=".12"/><inertia ixx=".0001" iyy=".0001" izz=".0001" ixy="0" ixz="0" iyz="0"/></inertial>
<visual><origin xyz="0 0 .025"/><geometry><cylinder radius=".022" length=".05"/></geometry><material name="metal"><color rgba=".25 .3 .35 1"/></material></visual>
<visual><origin xyz="0 0 .06"/><geometry><cylinder radius=".012" length=".04"/></geometry><material name="stem"><color rgba=".7 .75 .8 1"/></material></visual>
<visual><origin xyz="0 0 .085"/><geometry><cylinder radius=".029" length=".018"/></geometry><material name="rubber"><color rgba=".08 .1 .12 1"/></material></visual>
<visual><origin xyz="0 0 .101"/><geometry><cylinder radius=".039" length=".018"/></geometry><material name="rubber"/></visual>
</link>
<joint name="{pre}vacuum_joint" type="fixed"><parent link="{pre}tool0"/><child link="{pre}vacuum"/><origin xyz="0 0 0" rpy="0 0 0"/></joint>
<gazebo reference="{pre}vacuum_joint"><preserveFixedJoint>true</preserveFixedJoint></gazebo>
<gazebo><plugin filename="gz_ros2_control-system" name="gz_ros2_control::GazeboSimROS2ControlPlugin"><parameters>{P}/config/{typ}_controllers.yaml</parameters><ros><namespace>/{typ}</namespace></ros></plugin>
<plugin filename="gz-sim-detachable-joint-system" name="gz::sim::systems::DetachableJoint">
<parent_link>{pre}vacuum</parent_link><child_model>caja</child_model><child_link>body</child_link>
<attach_topic>/celda/{typ}/attach</attach_topic><detach_topic>/celda/{typ}/detach</detach_topic><output_topic>/celda/{typ}/attached</output_topic><initially_detached>true</initially_detached>
</plugin></gazebo>
<xacro:ur_ros2_control name="{typ}" tf_prefix="{pre}"{init}/>
</robot>'''
    write(P/'urdf'/f'{typ}.xacro',text)
    out=subprocess.check_output(['xacro',str(P/'urdf'/f'{typ}.xacro')],text=True)
    write(P/'urdf'/f'{typ}.urdf',out)
    return ET.fromstring(out)

def origin(el):
    T=np.eye(4)
    if el is not None:
        T[:3,3]=list(map(float,el.get('xyz','0 0 0').split()))
        T[:3,:3]=R.from_euler('xyz',list(map(float,el.get('rpy','0 0 0').split()))).as_matrix()
    return T

def chain(root,typ):
    bychild={j.find('child').get('link'):j for j in root.findall('joint')}
    ch=[]; c=typ+'_tool0'
    while c!='world':
        j=bychild[c]; ch.insert(0,j); c=j.find('parent').get('link')
    return ch

def fk(ch,q):
    T=np.eye(4); idx=0; pts=[]
    for j in ch:
        T=T@origin(j.find('origin'))
        if j.get('type') in ('revolute','continuous'):
            axis=np.array(list(map(float,j.find('axis').get('xyz').split())))
            A=np.eye(4); A[:3,:3]=R.from_rotvec(axis*q[idx]).as_matrix(); T=T@A;idx+=1
        pts.append(T[:3,3].copy())
    return T,pts
for typ in BASE:
    pre=typ+'_'
    write(P/'config'/f'{typ}_controllers.yaml',f'''/{typ}/controller_manager:
  ros__parameters:
    update_rate: 250
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
    joint_trajectory_controller:
      type: joint_trajectory_controller/JointTrajectoryController
/{typ}/joint_trajectory_controller:
  ros__parameters:
    joints: [{", ".join(pre+j for j in J)}]
    command_interfaces: [position]
    state_interfaces: [position, velocity]
    allow_partial_joints_goal: false
    constraints:
      goal_time: 3.0
      stopped_velocity_tolerance: 0.15
''')
    root=ur(typ); ch=chain(root,typ)
    targets=({'home':[-1.28,-.12,1.12],'pick_above':[-1.53,-.57,1.06],'pick':[-1.53,-.57,.963],'lift':[-1.53,-.57,1.06],
              'place_above':[-1.20,0,1.08],'place':[-1.20,0,.963],'retreat':[-1.20,0,1.08]} if typ=='ur3' else
             {'home':[1.24,-.13,1.38],'pick_above':[1.15,0,1.20],'pick':[1.15,0,.963],'lift':[1.15,0,1.30],
              'place_above':[2.02,-.53,1.14],'place':[2.02,-.53,.733],'retreat':[2.02,-.53,1.16]})
    seed=np.array([0,-1.4,1.8,-1.9,-1.57,0.])
    data={}
    desired=R.from_euler('xyz',[math.pi,0,0]).as_matrix()
    rng=np.random.default_rng(8)
    for name,tip in targets.items():
        pos=np.array(tip)+[0,0,.11]
        def fun(q):
            T,_=fk(ch,q)
            return np.r_[T[:3,3]-pos, .3*R.from_matrix(desired.T@T[:3,:3]).as_rotvec()]
        candidates=[]
        for sd in [seed]+[seed+rng.normal(0,1.7,6) for _ in range(7)]:
            res=least_squares(fun,np.clip(sd,-6.2,6.2),bounds=(-2*math.pi,2*math.pi),max_nfev=200,ftol=1e-10,gtol=1e-10,xtol=1e-10)
            if np.linalg.norm(fun(res.x))<1e-5:
                T,pts=fk(ch,res.x)
                # Prevent elbow / wrist from passing through the tabletop.
                if min(p[2] for p in pts[3:]) > .82:
                    candidates.append(res.x)
        if not candidates: raise RuntimeError((typ,name,'unreachable'))
        q=min(candidates,key=lambda q:np.linalg.norm(q-seed))
        data[name]={'q':q.tolist(),'tip':tip,'error_m':float(np.linalg.norm(fun(q)[:3]))}
        seed=q
    ur(typ,data['home']['q'])
    CONFIG[typ]=data
write(P/'config/waypoints.json',json.dumps(CONFIG,indent=2))

# Complete scene geometry. Coordinates in metres.
def mat(c):
    return f'<material><ambient>{c} 1</ambient><diffuse>{c} 1</diffuse><specular>.25 .25 .25 1</specular></material>'

def geo(kind,size):
    if kind=='box':return f'<box><size>{size}</size></box>'
    r,l=size.split();return f'<cylinder><radius>{r}</radius><length>{l}</length></cylinder>'

def shape(n,pos,size,c,kind='box',collision=False,rpy='0 0 0'):
    g=geo(kind,size); pose=f'{pos} {rpy}'
    return f'<visual name="{n}"><pose>{pose}</pose><geometry>{g}</geometry>{mat(c)}</visual>'+ (f'<collision name="{n}"><pose>{pose}</pose><geometry>{g}</geometry></collision>' if collision else '')

def fixed(n,parts):
    return f'<model name="{n}"><static>true</static><link name="body">{parts}</link></model>'
silver='.58 .64 .68';dark='.09 .13 .17';blue='.03 .28 .42';yellow='.95 .67 .04'

def table(n,x,y,l,w,top):
    v=shape('top',f'{x} {y} {top-.035}',f'{l} {w} .07',silver,collision=True)
    v+=shape('under',f'{x} {y} {top-.10}',f'{l-.04} {w-.04} .08',dark)
    for i,(dx,dy) in enumerate([(a,b) for a in [-l/2+.08,l/2-.08] for b in [-w/2+.08,w/2-.08]]):
        v+=shape(f'leg{i}',f'{x+dx} {y+dy} {(top-.12)/2}',f'.055 .055 {top-.12}',silver,collision=True)
        v+=shape(f'foot{i}',f'{x+dx} {y+dy} .025','.09 .09 .05',dark)
    v+=shape('brace',f'{x} {y+w/2-.08} .24',f'{l-.1} .04 .06',silver)
    return fixed(n,v)
models=[]
models.append(fixed('suelo',shape('slab','1 0 -.05','12 9 .10','.37 .40 .43',collision=True)))
# Floor tiles and aisle boundaries.
tiles=''
for x in range(-5,8):tiles+=shape('gx'+str(x),f'{x} 0 .001','.006 9 .002','.47 .49 .51')
for y in range(-4,5):tiles+=shape('gy'+str(y),f'1 {y} .001','12 .006 .002','.47 .49 .51')
for y in [-1.35,.65]:tiles+=shape('edge'+str(y),f'1 {y} .006','7.6 .045 .008',yellow)
for x in [-2.8,4.8]:tiles+=shape('end'+str(x),f'{x} -.35 .006','.045 2 .008',yellow)
# MiR straight route dashed markings.
for i in range(8):tiles+=shape(f'route{i}',f'{2+i*.29} -.98 .009','.15 .035 .008','.88 .9 .91')
models.append(fixed('marcas_suelo',tiles))
models.append(table('mesa_UR3',-1.38,-.72,1.0,.94,.8))
models.append(table('pedestal_UR5',1.3,-.53,.43,.43,.8))
models.append(table('mesa_entrega',4.4,-.53,.7,.85,.8))
belt=shape('belt','0 0 .765','2.9 .46 .07','.055 .09 .11',collision=True)
for y in [-.275,.275]:
    belt+=shape('rail'+str(y),f'0 {y} .73','3.02 .06 .15',silver)
    belt+=shape('stripe'+str(y),f'0 {y-.032} .755','2.85 .006 .026',blue)
for x in [-1.05,1.05]:
    for y in [-.22,.22]:
        belt+=shape('leg'+str(x)+str(y),f'{x} {y} .35','.055 .055 .70',silver,collision=True)
        belt+=shape('pad'+str(x)+str(y),f'{x} {y} .02','.11 .10 .04',dark)
for i in range(20):
    belt+=shape('roller'+str(i),f'{-1.38+i*.145} 0 .72','.038 .52','.36 .41 .45',kind='cylinder',rpy='1.5708 0 0')
belt+=shape('motor','-1.3 .40 .66','.08 .20',blue,kind='cylinder',rpy='1.5708 0 0')
belt+=shape('gearbox','-1.3 .32 .66','.15 .10 .16',dark)
for i,x in enumerate([-1.2,1.15]):
    belt+=shape('sensorpost'+str(i),f'{x} .31 .89','.018 .018 .24',silver)
    belt+=shape('sensor'+str(i),f'{x} .29 .98','.055 .045 .07',dark)
    belt+=shape('sensorLED'+str(i),f'{x} .265 .98','.014 .004 .014','.2 1 .1')
models.append(fixed('transportador',belt))
# Moving thin belt cleats, visuals only.
slats='<model name="belt_marks"><static>true</static>'
for i in range(7):
    slats+=f'<link name="mark_{i}"><pose>{-1.45+i*.1} 0 .803 0 0 0</pose>'+shape('line','0 0 0','.008 .44 .002','.22 .27 .29')+'</link>'
slats+='</model>';models.append(slats)
# Guarding behind the line, cabinet, stack light.
guard=''
for i,x in enumerate([-2.2,-.8,.6,2.0]):
    guard+=shape(f'post{i}',f'{x} .95 .72','.045 .045 1.44',yellow)
for i in range(34):
    guard+=shape(f'meshV{i}',f'{-2.2+i*.127} .95 .76','.004 .006 1.23','.19 .22 .24')
for i in range(11):
    guard+=shape(f'meshH{i}',f'-.1 .95 { .17+i*.115}','4.2 .006 .004','.19 .22 .24')
for z in [.12,1.42]:guard+=shape('bar'+str(z),f'-.1 .95 {z}','4.24 .04 .035',dark)
models.append(fixed('vallado',guard))
cab=shape('cabinet','-2.22 -.65 .47','.36 .42 .88','.80 .83 .85',collision=True)
cab+=shape('door','-2.22 -.866 .49','.31 .012 .75','.68 .73 .77')
cab+=shape('screen','-2.22 -.877 .72','.21 .012 .13',dark)
cab+=shape('display','-2.22 -.885 .72','.17 .005 .085','.03 .56 .71')
cab+=shape('stop','-2.15 -.90 .49','.035 .026','.85 .025 .025','cylinder',rpy='1.5708 0 0')
cab+=shape('stem','-2.22 -.65 1.02','.016 .22',dark,'cylinder')
for i,c in enumerate(['.08 .85 .25','.95 .65 .03','.65 .04 .04']):
    cab+=shape('lamp'+str(i),f'-2.22 -.65 {1.15+i*.055}','.035 .05',c,'cylinder')
models.append(fixed('armario_control',cab))
# Cardboard parcel, 180 x 160 x 160 mm, 250 g.
box=shape('carton','0 0 0','.18 .16 .16','.64 .39 .17',collision=True)
box+=shape('tape','0 0 .0808','.045 .161 .002','.82 .65 .38')
box+=shape('label','0 -.0808 .015','.10 .002 .065','.95 .95 .91')
for i in range(7):box+=shape('barcode'+str(i),f'{-.040+i*.006} -.082 .017',f'{.002 if i%3 else .003} .001 .04','.07 .08 .08')
models.append(f'<model name="caja"><pose>-1.53 -.57 .882 0 0 0</pose><link name="body"><inertial><mass>.25</mass><inertia><ixx>.00107</ixx><iyy>.0012</iyy><izz>.0012</izz></inertia></inertial>{box}</link></model>')
# MiR100 visual mesh from DFKI, adapted dynamic base for Harmonic.
mir_src=ROOT/'third_party/mir_robot/mir_description/meshes/visual/mir_100_base.stl'
if mir_src.exists():
    shutil.copy2(mir_src,P/'meshes/mir_100_base.stl')
elif not (P/'meshes/mir_100_base.stl').exists():
    raise RuntimeError('Falta la malla MiR100; consultar README y licencia DFKI')
m=f'<model name="mir100"><pose>2.02 -.53 .003 0 0 0</pose><link name="base_link"><inertial><pose>0 0 .16 0 0 0</pose><mass>77</mass><inertia><ixx>1.1</ixx><iyy>3.6</iyy><izz>4.5</izz></inertia></inertial><visual name="body_mesh"><geometry><mesh><uri>file://{P}/meshes/mir_100_base.stl</uri></mesh></geometry>{mat(".86 .88 .90")}</visual>'
m+=shape('body_collision','0 0 .20','.86 .56 .25',silver,collision=True).replace('<visual name="body_collision">','<visual name="body_collision"><visibility_flags>0</visibility_flags>')
m+=shape('bumper','0 0 .12','.9 .58 .07',dark)
m+=shape('deck','0 0 .535','.70 .48 .07','.34 .4 .44',collision=True)
for x in [-.26,.26]:
    for y in [-.17,.17]:m+=shape('rack'+str(x)+str(y),f'{x} {y} .415','.035 .035 .17',silver)
for y in [-.293,.293]:m+=shape('led'+str(y),f'0 {y} .24','.53 .008 .018','.1 .8 .9')
for x,y in [(.38,.23),(-.38,-.23)]:
    m+=shape('scanner'+str(x),f'{x} {y} .18','.04 .065',dark,'cylinder')
    m+=shape('scannerlens'+str(x),f'{x} {y} .20','.041 .02','.02 .19 .26','cylinder')
# Low friction support spheres act as caster contacts.
for i,(x,y) in enumerate([(a,b) for a in [-.31,.31] for b in [-.2,.2]]):
    m+=f'<collision name="caster{i}"><pose>{x} {y} .0625 0 0 0</pose><geometry><sphere><radius>.0625</radius></sphere></geometry><surface><friction><ode><mu>.001</mu><mu2>.001</mu2></ode></friction></surface></collision>'
    m+=shape('casterV'+str(i),f'{x} {y} .065','.055 .026',dark,'cylinder',rpy='1.5708 0 0')
m+='</link>'
for name,y in [('left',.2226),('right',-.2226)]:
    m+=f'<link name="{name}_wheel"><pose>0 {y} .0625 1.57079632679 0 0</pose><inertial><mass>1</mass><inertia><ixx>.001</ixx><iyy>.001</iyy><izz>.0019</izz></inertia></inertial>'+shape('wheel','0 0 0','.0625 .032',dark,'cylinder',collision=True)+f'</link><joint name="{name}_joint" type="revolute"><parent>base_link</parent><child>{name}_wheel</child><axis><xyz expressed_in="__model__">0 1 0</xyz><limit><lower>-1e16</lower><upper>1e16</upper></limit></axis></joint>'
m+='''<plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive"><left_joint>left_joint</left_joint><right_joint>right_joint</right_joint><wheel_separation>.4452</wheel_separation><wheel_radius>.0625</wheel_radius><topic>/celda/mir/cmd_vel</topic><odom_topic>/celda/mir/odom</odom_topic><max_linear_acceleration>.25</max_linear_acceleration><min_linear_acceleration>-.25</min_linear_acceleration></plugin>
<plugin filename="gz-sim-detachable-joint-system" name="gz::sim::systems::DetachableJoint"><parent_link>base_link</parent_link><child_model>caja</child_model><child_link>body</child_link><attach_topic>/celda/mir/attach</attach_topic><detach_topic>/celda/mir/detach</detach_topic><output_topic>/celda/mir/attached</output_topic><initially_detached>true</initially_detached></plugin></model>'''
models.append(m)
for typ in BASE:
    sdf=subprocess.check_output(['gz','sdf','-p',str(P/'urdf'/f'{typ}.urdf')],text=True)
    model=ET.fromstring(sdf).find('model')
    models.append(ET.tostring(model,encoding='unicode'))
world='''<sdf version="1.9"><world name="celda">
<physics name="physics" type="ignored"><max_step_size>.004</max_step_size><real_time_factor>1</real_time_factor></physics>
<gravity>0 0 -9.81</gravity>
<plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
<plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
<plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
<plugin filename="libCellEnvironment.so" name="celda::CellEnvironment"/>
<scene><ambient>.65 .65 .65 1</ambient><background>.72 .78 .84 1</background><shadows>true</shadows></scene>
<light type="directional" name="key"><pose>0 0 8 0 0 0</pose><diffuse>.95 .95 .95 1</diffuse><specular>.25 .25 .25 1</specular><direction>-.3 .4 -1</direction><cast_shadows>true</cast_shadows></light>
<light type="directional" name="fill"><diffuse>.45 .48 .52 1</diffuse><direction>.4 -.3 -1</direction><cast_shadows>false</cast_shadows></light>
<gui fullscreen="false">
<plugin filename="MinimalScene" name="3D View"><gz-gui><title>Celda robotica UR3 UR5 MiR100</title><property type="bool" key="showTitleBar">false</property><property type="string" key="state">docked</property></gz-gui><engine>ogre2</engine><scene>scene</scene><ambient_light>.65 .65 .65</ambient_light><background_color>.72 .78 .84</background_color><camera_pose>5 -7 5 0 .48 2.08</camera_pose></plugin>
<plugin filename="GzSceneManager" name="Scene Manager"><gz-gui><property key="state" type="string">floating</property><property key="showTitleBar" type="bool">false</property><property key="width" type="double">5</property><property key="height" type="double">5</property></gz-gui></plugin>
<plugin filename="InteractiveViewControl" name="View control"><gz-gui><property key="state" type="string">floating</property><property key="showTitleBar" type="bool">false</property><property key="width" type="double">5</property><property key="height" type="double">5</property></gz-gui></plugin>

<plugin filename="WorldControl" name="World control"><gz-gui><title>Simulacion</title><property type="bool" key="showTitleBar">false</property><property type="string" key="state">floating</property><property type="double" key="width">220</property><property type="double" key="height">70</property><anchors target="3D View"><line own="left" target="left"/><line own="bottom" target="bottom"/></anchors></gz-gui><play_pause>true</play_pause><step>true</step><start_paused>false</start_paused></plugin>
<plugin filename="WorldStats" name="World stats"><gz-gui><property type="string" key="state">floating</property><property type="double" key="width">240</property><property type="double" key="height">110</property><anchors target="3D View"><line own="right" target="right"/><line own="bottom" target="bottom"/></anchors></gz-gui><sim_time>true</sim_time><real_time>true</real_time><real_time_factor>true</real_time_factor></plugin>
</gui>'''
world+=''.join(models)+'</world></sdf>'
tree=ET.fromstring(world)
for model in tree.findall('.//model'):
    for pl in list(model.findall('plugin')):
        if 'DetachableJoint' in pl.get('name',''):model.remove(pl)
w=tree.find('world')
w.append(ET.fromstring('<plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>'))
w.append(ET.fromstring('<model name="evidence_camera"><static>true</static><pose>4 -6 4 0 .47 2.05</pose><link name="camera"><sensor name="overview" type="camera"><always_on>true</always_on><update_rate>1</update_rate><topic>/celda/camera</topic><camera><horizontal_fov>1.15</horizontal_fov><image><width>1000</width><height>625</height><format>R8G8B8</format></image><clip><near>.1</near><far>100</far></clip></camera></sensor></link></model>'))
gui=tree.find('.//gui')
for pl in list(gui.findall('plugin')):
    if pl.get('filename')=='WorldStats':gui.remove(pl)
    if pl.get('filename')=='MinimalScene':
        pl.find('camera_pose').text='3.4 -4.8 3.5 0 .49 2.10'
        ET.SubElement(pl,'grid').text='false'
world=ET.tostring(tree,encoding='unicode')
write(P/'worlds/celda.sdf',world)
print('Escena generda. IK error máximo:',max(v['error_m'] for d in CONFIG.values() for v in d.values()))

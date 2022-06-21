#!/usr/bin/env python
import rospy
import time
from smach import State, StateMachine
from smach_ros import IntrospectionServer

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal 
from geometry_msgs.msg import PoseWithCovarianceStamped
from jinko_service_msg.srv import jinko_service_msg, jinko_service_msgResponse 

## Puntos de la casa
waypoints = [
    ['PUERTA', (2.5, 0.0, 0.0), (0.0, 0.0, 2.9, 0.0)],
    ['MESA', (6.5, -0.5, 0.0), (0.0, 0.0, 3, 0.0)],
    ['ZONA_DE_JUEGOS', (6.5, -3.0, 0.0), (0.0, 0.0, 1, 0.0)],
]

## Se guardara la posicion a la que se deberia mover el robot para compararla posteriormente con la real
donde_deberia_estar_el_robot = [0.0, 0.0]

## Si la posicion donde deberia estar y la real son similares (+-10cm) seria True
esta_donde_deberia = False
desviacion = 0.1

## variable para comprobar si ya se ha realizado la funcion comprobarPosicion
se_ha_comprobado = False

## Descripcion de los estados
class PowerOnRobot(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded','aborted'])

    def execute(self, userdata):
        rospy.loginfo("Encendiendo el Robot")
        time.sleep(2)
        return 'succeeded'


class WaitingOrder(State):
    def __init__(self, order_state):
        State.__init__(self, outcomes=['succeeded', 'aborted'], input_keys=[''], output_keys=[''])
        self.order = order_state

    def execute(self, userdata):
        if self.order == 1:
            return 'succeeded'
        else:
            return 'aborted'



class Navigate(State):
    def __init__(self, position, orientation, place):
        State.__init__(self, outcomes=['succeeded', 'aborted'], input_keys=[''], output_keys=[''])
        self._position =position
        self._orientation =orientation
        self._place = place
        self._move_base = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        rospy.loginfo("Activando el cliente de navegacion..")
        self._move_base.wait_for_server(rospy.Duration(15))


    def execute(self, userdata):
        time.sleep(2)
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        rospy.loginfo(self._position)
        goal.target_pose.pose.position.x = self._position[0]
        goal.target_pose.pose.position.y = self._position[1]
        goal.target_pose.pose.position.z = self._position[2]
        goal.target_pose.pose.orientation.x = self._orientation[0]
        goal.target_pose.pose.orientation.y = self._orientation[1]
        goal.target_pose.pose.orientation.z = self._orientation[2]
        goal.target_pose.pose.orientation.w = self._orientation[3]

        rospy.loginfo("ROBOT %s" %(self._place))
        # sends the goal
        self._move_base.send_goal(goal)
        self._move_base.wait_for_result()
        # Comprobamos el estado de la navegacion
        nav_state = self._move_base.get_state()
        rospy.loginfo("[Result] State: %d" % (nav_state))
        nav_state = 3

        if nav_state == 3:
            return 'succeeded'
        else:
            return 'aborted'
"""
class Charge(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded','aborted'], input_keys=['input'], output_keys=[''])

    def execute(self, userdata):
        print("Revisando la carga de la bateria...")
        if userdata.input == 1:
            print("Robot cargado")
            return 'succeeded'
        else:
            print("Robot sin carga")
            return 'aborted'
"""

def my_callback(request):
    global donde_deberia_estar_el_robot
    global esta_donde_deberia
    global se_ha_comprobado

    meta = request.destino
    metaX = request.coordenadaX
    metaY = request.coordenadaY
    if meta == 1:
        sm_descansar = StateMachine(outcomes=['succeeded', 'aborted'])
        sm_descansar.userdata.sm_input = meta

        with sm_descansar:
            StateMachine.add('POWER_ON', PowerOnRobot(),
                             transitions={'succeeded': 'WAITING_ORDER', 'aborted': 'aborted'})
            StateMachine.add('WAITING_ORDER', WaitingOrder(1),
                             transitions={'succeeded': 'PUERTA', 'aborted': 'aborted'})
            StateMachine.add(waypoints[0][0], Navigate(waypoints[0][1], waypoints[0][2], waypoints[0][0]),
                             transitions={'succeeded': 'succeeded', 'aborted': 'WAITING_ORDER'})

            # Guarda donde deberia haber ido el robot
            donde_deberia_estar_el_robot = [waypoints[0][1][0], waypoints[0][1][1]]

        intro_server = IntrospectionServer('JinkoBot', sm_descansar, '/SM_ROOT')
        intro_server.start()

        # Ejecutamos la maquina de estados
        sm_outcome = sm_descansar.execute()
        print(sm_outcome)
        intro_server.stop()

    elif meta == 2:
        sm_jugar = StateMachine(outcomes=['succeeded', 'aborted'])
        sm_jugar.userdata.sm_input = meta

        with sm_jugar:
            StateMachine.add('POWER_ON', PowerOnRobot(),
                             transitions={'succeeded': 'WAITING_ORDER', 'aborted': 'aborted'})
            StateMachine.add('WAITING_ORDER', WaitingOrder(1),
                             transitions={'succeeded': 'ZONA_DE_JUEGOS', 'aborted': 'aborted'})
            StateMachine.add(waypoints[2][0], Navigate(waypoints[2][1], waypoints[2][2], waypoints[2][0]),
                             transitions={'succeeded': 'succeeded', 'aborted': 'WAITING_ORDER'})

            # Guarda donde deberia haber ido el robot
            dondeDeberiaEstarElRobot = [waypoints[2][1][0], waypoints[2][1][1]]

        intro_server = IntrospectionServer('JinkoBot', sm_jugar, '/SM_ROOT')
        intro_server.start()

        # Ejecutamos la maquina de estados
        sm_outcome = sm_jugar.execute()
        print(sm_outcome)
        intro_server.stop()

    elif meta == 3:
        sm_estudiar = StateMachine(outcomes=['succeeded', 'aborted'])
        sm_estudiar.userdata.sm_input = meta

        with sm_estudiar:
            StateMachine.add('POWER_ON', PowerOnRobot(),
                             transitions={'succeeded': 'WAITING_ORDER', 'aborted': 'aborted'})
            StateMachine.add('WAITING_ORDER', WaitingOrder(1),
                             transitions={'succeeded': 'MESA', 'aborted': 'aborted'})
            StateMachine.add(waypoints[1][0], Navigate(waypoints[1][1], waypoints[1][2], waypoints[1][0]),
                             transitions={'succeeded': 'succeeded', 'aborted': 'WAITING_ORDER'})

            # Guarda donde deberia haber ido el robot
            dondeDeberiaEstarElRobot = [waypoints[1][1][0], waypoints[1][1][1]]

        intro_server = IntrospectionServer('JinkoBot', sm_estudiar, '/SM_ROOT')
        intro_server.start()

        # Ejecutamos la maquina de estados
        sm_outcome = sm_estudiar.execute()
        print(sm_outcome)
        intro_server.stop()

    elif meta == 4:
        sm_tour = StateMachine(outcomes=['succeeded', 'aborted'])
        sm_tour.userdata.sm_input = meta

        with sm_tour:
            StateMachine.add('POWER_ON', PowerOnRobot(),
                             transitions={'succeeded': 'WAITING_ORDER', 'aborted': 'aborted'})
            StateMachine.add('WAITING_ORDER', WaitingOrder(1),
                             transitions={'succeeded': 'MESA', 'aborted': 'aborted'})
            StateMachine.add(waypoints[1][0], Navigate(waypoints[1][1], waypoints[1][2], waypoints[1][0]),
                             transitions={'succeeded': 'ZONA_DE_JUEGOS', 'aborted': 'WAITING_ORDER'})
            StateMachine.add(waypoints[2][0], Navigate(waypoints[2][1], waypoints[2][2], waypoints[2][0]),
                             transitions={'succeeded': 'PUERTA', 'aborted': 'WAITING_ORDER'})
            StateMachine.add(waypoints[0][0], Navigate(waypoints[0][1], waypoints[0][2], waypoints[0][0]),
                             transitions={'succeeded': 'succeeded', 'aborted': 'WAITING_ORDER'})

            # Guarda donde deberia haber ido el robot
            dondeDeberiaEstarElRobot = [waypoints[0][1][0], waypoints[0][1][1]]

        intro_server = IntrospectionServer('JinkoBot', sm_tour, '/SM_ROOT')
        intro_server.start()

        # Ejecutamos la maquina de estados
        sm_outcome = sm_tour.execute()
        print(sm_outcome)
        intro_server.stop()

    elif meta == 5:
        exit()

    elif meta == 0: # Navegar a coordenadas variables que aporte el usuario desde coordenadaX y coordenadaY al llamar a /jinko_navigation
        sm_coordenadas = StateMachine(outcomes=['succeeded', 'aborted'])
        sm_coordenadas.userdata.sm_input = [metaX, metaY]

        with sm_coordenadas:
            StateMachine.add('POWER_ON', PowerOnRobot(),
                             transitions={'succeeded': 'WAITING_ORDER', 'aborted': 'aborted'})
            StateMachine.add('WAITING_ORDER', WaitingOrder(1),
                             transitions={'succeeded': 'NAVIGATE_TO_UNKNOWN', 'aborted': 'aborted'})
            StateMachine.add('NAVIGATE_TO_UNKNOWN', Navigate((metaX, metaY, 0.0), (0.0, 0.0, 0.7, 0.0), 'PUNTO'),
                             transitions={'succeeded': 'succeeded', 'aborted': 'WAITING_ORDER'})

        intro_server = IntrospectionServer('JinkoBot', sm_coordenadas, '/SM_ROOT')
        intro_server.start()
        # Guarda donde deberia haber ido el robot
        donde_deberia_estar_el_robot = [metaX, metaY]

        # Ejecutamos la maquina de estados
        sm_outcome = sm_coordenadas.execute()
        print(sm_outcome)
        intro_server.stop()

    else:
        print("Introduce valores validos de destino (0, 1, 2, 3, 4, 5)")

    # COMPROBACION POSICION REAL DEL ROBOT
    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, comprobacionPosicion)

    while not se_ha_comprobado: # Espera a que se haya comprobado
        1+1

    response = jinko_service_msgResponse()
    response.success = esta_donde_deberia
    se_ha_comprobado = False
    return response


def comprobacionPosicion(poseWithCovariance):
    global donde_deberia_estar_el_robot
    global esta_donde_deberia
    global desviacion
    global se_ha_comprobado

    X = poseWithCovariance.pose.pose.position.x
    Y = poseWithCovariance.pose.pose.position.y

    if donde_deberia_estar_el_robot[0] <= float(X)+desviacion and donde_deberia_estar_el_robot[0] >= float(X)-desviacion and donde_deberia_estar_el_robot[1] <= float(Y)+desviacion and donde_deberia_estar_el_robot[1] >= float(Y)-desviacion:
        esta_donde_deberia = True
    else:
        esta_donde_deberia = False
    se_ha_comprobado = True
    # print(str(donde_deberia_estar_el_robot)+" y "+str(X)+" "+str(Y))


class main():
    def __init__(self):

        # Iniciar servicio
        rospy.init_node('move_base_action_client', anonymous=False)
        rospy.Service('/jinko_navigation', jinko_service_msg, my_callback)
        rospy.spin()


if __name__=='__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo(" Testeo JINKOBOT finalizado")

#!/usr/bin/env python
import rospy
import time
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal 
from geometry_msgs.msg import PoseWithCovarianceStamped
from jinko_service_msg.srv import jinko_service_msg, jinko_service_msgResponse 

## Se guardara la posicion a la que se deberia mover el robot para compararla posteriormente con la real
donde_deberia_estar_el_robot = [0.0, 0.0]

## Si la posicion donde deberia estar y la real son similares (+-10cm) seria True
esta_donde_deberia = False
desviacion = 0.1

## variable para comprobar si ya se ha realizado la funcion comprobarPosicion
se_ha_comprobado = False

class Navigate():
    def __init__(self, position, orientation):
        self._position =position
        self._orientation =orientation
        self._move_base = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        rospy.loginfo("Activando el cliente de navegacion..")
        self._move_base.wait_for_server(rospy.Duration(5))


    def execute(self):
        # time.sleep(2)
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

        # sends the goal
        self._move_base.send_goal(goal)
        self._move_base.wait_for_result()
        nav_state = self._move_base.get_state()
        rospy.loginfo("[Result] State: %d" % (nav_state))
        nav_state = 3

        if nav_state == 3:
            return True
        else:
            return False


def my_callback(request):
    global donde_deberia_estar_el_robot
    global esta_donde_deberia
    global se_ha_comprobado

    metaX = request.coordenadaX
    metaY = request.coordenadaY

    navigate = Navigate((metaX, metaY, 0.0), (0.0, 0.0, 0.5, 0.0))
    navigate.execute()

    # Guarda donde deberia haber ido el robot
    donde_deberia_estar_el_robot = [metaX, metaY]

    # COMPROBACION POSICION REAL DEL ROBOT
    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, comprobacionPosicion)

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

class main():
    def __init__(self):
        rospy.init_node('Orientation Server', anonymous=False)
        rospy.Service('/jinko_navigation', jinko_service_msg, my_callback)
        rospy.spin()


if __name__=='__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Ha habido un error")

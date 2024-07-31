#!/usr/bin/env python3
import time

import rclpy

from rclpy.node import Node
from geometry_msgs.msg import Twist


PEDAL_MSGS = {
    'Current Speed': '',                    # Текущая скорость автомобиля от блока управления
    'Current Clutch': '',                   # Текущее состояние педали сцепления от блока управления
    'Current Break': '',                    # Текущее состояние педали тормоза от блока управления

    'Required Clutch': '',                  # Требуемое состояние сцепления от компьютера
    'Required Break': '',                   # Требуемое состояние тормоза от компьютера
}

STEERING_MSGS = {
    'Current Steering': '',                 # Текущий угол поворота колес от блока управления
    'Current Ignition': '',                 # Текущее состояние зажигания от блока управления

    'Required Steering': '',                # Требуемый угол поворота колес от компьютера

    'Ignition (EPS) Error': '',             # Двигатель не заведен, эур не работает
    'Encoder Connection Error': '',         # Потеря сигнала от энкодера руля
    'Steering Timeout Error': '',           # Руль не выходит в заданное положение за время таймаута
    'Steering Jump Over Error': '',         # Прокрутка энкодера больше одного оборота
}

TRANSMISSION_MSGS = {
    'Current Gear': '',                     # Текущая включенная передача от блока управления

    'Required Gear': '',                    # Требуемая передача от компьютера

    'FPotentiometer Connection Error': '',  # Потеря сигнала потенциометра актуатора переключения вперед-назад
    'FPotentiometer Range Error': '',       # Выход из допустимых диапазонов потенциометра актуатора переключения вперед-назад
    'FTransmission Activation Error': '',   # Ошибка включения передачи актуатора переключения вперед-назад

    'SPotentiometer Connection Error': '',  # Потеря сигнала потенциометра актуатора переключения влево-вправо
    'SPotentiometer Range Error': '',       # Выход из допустимых диапазонов потенциометра актуатора переключения влево-вправо
    'STransmission Activation Error': '',   # Ошибка включения передачи актуатора переключения влево-вправо
}


class MovementController(Node):
    def __init__(self, node_name: str):
        super().__init__(node_name)

        self.declare_parameter('cmd_topic', '/cmd_vel')
        self.declare_parameter('frequency', 30)
        self.declare_parameter('reset_errors', False)

        cmd_topic = self.get_parameter('cmd_topic').get_parameter_value().string_value
        frequency = self.get_parameter('frequency').get_parameter_value().integer_value

        self.cmdSub = self.create_subscription(Twist,
                                               cmd_topic,
                                               self.cmd_callback,
                                               10)

        self.mainTimer = self.create_timer(1 / frequency, self.timer_callback)
        self.clutchTimer = self.create_timer(1 / frequency, self.clutch_callback)
        self.breakTimer = self.create_timer(1 / frequency, self.break_callback)
        self.transmissionTimer = self.create_timer(1 / frequency, self.transmission_callback)

        self.clutchTimer.cancel()
        self.breakTimer.cancel()

        self.canTimer = self.create_timer(1 / frequency, self.can_callback)

        self.currentBreakState = 0
        self.currentClutchState = 0
        self.currentSteeringAngle = 0
        self.currentTransmissionState = 0
        self.currentIgnitionState = 0

        self.reqBreakState = 0
        self.reqClutchState = 0
        self.reqSteeringAngle = 0
        self.reqTransmissionState = 0

        self.errors = []

        self.cmdData = Twist()

    def cmd_callback(self, msg):
        self.cmdData = msg

    def timer_callback(self):
        if self.get_parameter('reset_errors').get_parameter_value().bool_value:
            self.reset_errors()
            self.set_parameters([rclpy.Parameter('reset_errors', rclpy.Parameter.Type.BOOL, False)])

        if self.errors is []:
            if self.currentIgnitionState:
                speed = self.cmdData.linear.x
                steer = self.cmdData.angular.z
                # Отправить требуемое положение руля
                if speed > 0:
                    self.reqTransmissionState = 1
                elif speed < 0:
                    self.reqTransmissionState = -1
                else:
                    self.reqTransmissionState = 0
            else:
                self.press_clutch()
                self.press_break()
                self.get_logger().info("The car is not started! Waiting ignition...")
                time.sleep(3)
        else:
            self.get_logger().error("Check errors!")

    def transmission_callback(self):
        if self.currentClutchState == 0 and self.reqTransmissionState != self.currentTransmissionState:
            self.press_clutch()
        if self.currentClutchState == 1 and self.reqTransmissionState != self.currentTransmissionState:
            # Отправить требуемую передачу для переключения
            if self.currentBreakState == 1:
                self.release_break()
        if self.currentClutchState == 1 and self.reqTransmissionState == self.currentTransmissionState:
            if self.clutchTimer.is_canceled():
                txt = f"{self.currentTransmissionState}" if self.currentTransmissionState != 0 else "N"
                txt = "R" if self.currentTransmissionState == -1 else txt
                self.get_logger().info(f"{txt} transmission is enabled")
            self.release_clutch()

    def clutch_callback(self):
        # Отправить требуемое состояние сцепления
        if self.currentClutchState == self.reqClutchState:
            txt = "Pressed" if self.currentClutchState == 1 else "Released"
            self.get_logger().info(f"Clutch is {txt}")
            self.clutchTimer.cancel()

    def break_callback(self):
        # Отправить требуемое состояние тормоза
        if self.currentBreakState == self.reqBreakState:
            txt = "Pressed" if self.currentBreakState == 1 else "Released"
            self.get_logger().info(f"Break is {txt}")
            self.breakTimer.cancel()

    def can_callback(self):
        # Получение всех параметров автомобиля
        pass

    def press_clutch(self):
        if self.currentClutchState == 0:
            if self.clutchTimer.is_canceled():
                self.reqClutchState = 1
                self.clutchTimer.reset()

    def press_break(self):
        if self.currentBreakState == 0:
            if self.breakTimer.is_canceled():
                self.reqBreakState = 1
                self.breakTimer.reset()

    def release_clutch(self):
        if self.currentClutchState == 1:
            if self.clutchTimer.is_canceled():
                self.reqClutchState = 0
                self.clutchTimer.reset()

    def release_break(self):
        if self.currentBreakState == 1:
            if self.breakTimer.is_canceled():
                self.reqBreakState = 0
                self.breakTimer.reset()

    def reset_errors(self):
        self.errors = []


def main():
    rclpy.init()
    node = MovementController("movement_controller")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <iostream>
#include <cmath>

#define TIME_STEP 64
#define MAX_SPEED 6.28

using namespace webots;
using namespace std;

int main()
{
    Robot *robot = new Robot();

    // Sensores de suelo del E-puck (gs0=izquierda, gs2=derecha, gs1=centro)
    DistanceSensor *leftSensor = robot->getDistanceSensor("gs0");
    DistanceSensor *rightSensor = robot->getDistanceSensor("gs2");
    leftSensor->enable(TIME_STEP);
    rightSensor->enable(TIME_STEP);

    Motor *leftMotor = robot->getMotor("left wheel motor");
    Motor *rightMotor = robot->getMotor("right wheel motor");
    leftMotor->setPosition(INFINITY);
    rightMotor->setPosition(INFINITY);
    leftMotor->setVelocity(0.0);
    rightMotor->setVelocity(0.0);

    // Parámetros del controlador PID (BALANCEADOS para giros fuertes pero controlados)
    double Kp = 3.5;    // Ganancia proporcional MODERADA-ALTA
    double Ki = 0.0005; // Ganancia integral baja para evitar acumulación excesiva
    double Kd = 1.2;    // Ganancia derivativa para suavizar transiciones

    // Variables del PID
    double integral = 0.0;
    double previousError = 0.0;
    double lastValidError = 0.0; // Para mantener última dirección conocida

    // Parámetros de velocidad
    double baseSpeed = 4.0;     // velocidad base para avanzar bien
    double maxCorrection = 3.5; // corrección moderada para no parar completamente

    // Normalización de sensores de suelo E-puck
    // Los sensores gs dan ~1000 en blanco y ~300-400 en negro
    double sensorMax = 1000.0; // valor máximo del sensor (superficie blanca/clara)
    double sensorMin = 300.0;  // valor aproximado del sensor (superficie negra)

    while (robot->step(TIME_STEP) != -1)
    {
        double leftValue = leftSensor->getValue();
        double rightValue = rightSensor->getValue();

        // Normalizar valores de sensores a rango [0, 1]
        // IMPORTANTE: Invertir para que 1 = negro (línea), 0 = blanco (fondo)
        // Tus sensores dan 1000 en blanco y ~0 en negro
        double leftNorm = 1.0 - ((leftValue - sensorMin) / (sensorMax - sensorMin));
        double rightNorm = 1.0 - ((rightValue - sensorMin) / (sensorMax - sensorMin));

        // Limitar valores normalizados
        leftNorm = max(0.0, min(1.0, leftNorm));
        rightNorm = max(0.0, min(1.0, rightNorm));

        // Calcular ERROR del PID
        // Error positivo = sensor der ve más negro -> girar izquierda
        // Error negativo = sensor izq ve más negro -> girar derecha
        double error = rightNorm - leftNorm;

        // Si ambos sensores ven blanco (perdió la línea), usar último error conocido
        if (leftNorm < 0.1 && rightNorm < 0.1)
        {
            // Perdió la línea completamente - mantener última dirección moderadamente
            error = lastValidError * 1.2; // Amplificar ligeramente para seguir buscando
        }
        else
        {
            // Hay al menos un sensor en la línea - actualizar
            lastValidError = error;
        }

        // Componente PROPORCIONAL
        double P = Kp * error;

        // Componente INTEGRAL (acumula error en el tiempo)
        integral += error * (TIME_STEP / 1000.0);
        // Anti-windup: limitar la integral
        integral = max(-50.0, min(50.0, integral));
        double I = Ki * integral;

        // Componente DERIVATIVA (rate of change del error)
        double derivative = (error - previousError) / (TIME_STEP / 1000.0);
        double D = Kd * derivative;

        // Salida total del PID
        double pidOutput = P + I + D;

        // Limitar corrección
        pidOutput = max(-maxCorrection, min(maxCorrection, pidOutput));

        // Calcular velocidades de las ruedas
        // Si pidOutput > 0: girar izquierda (reducir velocidad izq, aumentar der)
        // Si pidOutput < 0: girar derecha (aumentar velocidad izq, reducir der)
        double leftSpeed = baseSpeed - pidOutput;
        double rightSpeed = baseSpeed + pidOutput;

        // Limitar velocidades al rango permitido
        leftSpeed = max(0.0, min(MAX_SPEED, leftSpeed));
        rightSpeed = max(0.0, min(MAX_SPEED, rightSpeed));

        // Aplicar velocidades
        leftMotor->setVelocity(leftSpeed);
        rightMotor->setVelocity(rightSpeed);

        // Actualizar error previo
        previousError = error;

        // Debug info
        cout << "L:" << leftValue << " R:" << rightValue
             << " | Error:" << error
             << " | P:" << P << " I:" << I << " D:" << D
             << " | PID:" << pidOutput
             << " | Vel L:" << leftSpeed << " R:" << rightSpeed << endl;
    }

    delete robot;
    return 0;
}

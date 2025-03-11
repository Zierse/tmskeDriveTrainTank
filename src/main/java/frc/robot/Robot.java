// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.core.CorePigeon2;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  //Definição de Variaveis
  SparkMax motorSuperiorDireitoLider; 
  SparkMax motorInferiorDireitoSeg;
  SparkMax motorSuperiorEsquerdoLider;
  SparkMax motorInferiorEsquerdoSeg;
  Joystick logitech;

  private static final double AJUSTE_ANALOG = 0.5;
  private static final int CURRENT_LIMIT = 30;

  private static final int JOYSTICK_PORT = 0;

  private static final int MOTOR_SUPERIOR_ESQUERDO_LIDER = 2;
  private static final int MOTOR_INFERIOR_ESQUERDO_SEG = 3;
  private static final int MOTOR_SUPERIOR_DIREITO_LIDER = 4;
  private static final int MOTOR_INFERIOR_DIREITO_SEG = 5;

  // Definindo constantes para os índices dos eixos
  private static final int AXIS_GATILHO_ESQUERDO = 2;
  private static final int AXIS_GATILHO_DIREITO = 3;
  private static final int AXIS_ANALOGICO_ESQUERDO = 0;

  private CorePigeon2 gyro;

  public Robot() {

    gyro = new CorePigeon2(21); //Definindo o Gyro como um CorePigeon2 para um Pigeon 2.0

    motorSuperiorEsquerdoLider = new SparkMax(MOTOR_SUPERIOR_ESQUERDO_LIDER, MotorType.kBrushless); //Estanciando variaveis com os parametros do motores.
    motorInferiorEsquerdoSeg = new SparkMax(MOTOR_INFERIOR_ESQUERDO_SEG, MotorType.kBrushless); //Estanciando variaveis com os parametros do motores.
    motorSuperiorDireitoLider = new SparkMax(MOTOR_SUPERIOR_DIREITO_LIDER, MotorType.kBrushless); //Estanciando variaveis com os parametros do motores.
    motorInferiorDireitoSeg = new SparkMax(MOTOR_INFERIOR_DIREITO_SEG, MotorType.kBrushless); //Estanciando variaveis com os parametros do motores.

    SparkMaxConfig configGlobal = new SparkMaxConfig(); //Estaciando as configurações para motores
    SparkMaxConfig configMotorDireitoLider = new SparkMaxConfig();
    SparkMaxConfig configMotorDireitoSeg = new SparkMaxConfig();
    SparkMaxConfig configMotorEsquerdoSeg = new SparkMaxConfig();

    //Definindo Configurações
    configGlobal
        .smartCurrentLimit(CURRENT_LIMIT)
        .idleMode(IdleMode.kBrake);

    configMotorDireitoLider
        .apply(configGlobal)
        .inverted(true);

    configMotorEsquerdoSeg
        .apply(configGlobal)
        .inverted(false)
        .follow(motorSuperiorEsquerdoLider);

    configMotorDireitoSeg
        .apply(configGlobal)
        .follow(motorSuperiorDireitoLider);

    //Parametrizando as configurações
        motorSuperiorEsquerdoLider.configure(configGlobal, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motorInferiorEsquerdoSeg.configure(configMotorEsquerdoSeg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motorSuperiorDireitoLider.configure(configMotorDireitoLider, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motorInferiorDireitoSeg.configure(configMotorDireitoSeg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        logitech = new Joystick(JOYSTICK_PORT); //Estaciando o Joystick
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {

    StatusSignal<Angle> yaw = gyro.getYaw(); //Angulo de rotação do eixo Z
    StatusSignal<Angle> pitch = gyro.getPitch(); //Angulo de rotação do eixo Y
    StatusSignal<Angle> roll = gyro.getRoll(); //Angulo de rotacao do eixo X
    
    double gatEsquerdo = -logitech.getRawAxis(AXIS_GATILHO_ESQUERDO);
    double gatDireito = -logitech.getRawAxis(AXIS_GATILHO_DIREITO);

    double analogEsquerdo = logitech.getRawAxis(AXIS_ANALOGICO_ESQUERDO);

    double valorGat = gatDireito - gatEsquerdo;
    double ajusteAnalogEsquerdo = analogEsquerdo * AJUSTE_ANALOG;

    motorSuperiorEsquerdoLider.set(valorGat + ajusteAnalogEsquerdo);
    motorSuperiorDireitoLider.set(valorGat - ajusteAnalogEsquerdo);


    //SmartDashboard.putNumber("Rotacao do Eixo Z Yaw", yaw.getValueAsDouble());
    //SmartDashboard.putNumber("Rotacao do Eixo Y Pitch", pitch.getValueAsDouble());
    //SmartDashboard.putNumber("Rotacao do Eixo X Roll", roll.getValueAsDouble());

    

  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}
 
  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}

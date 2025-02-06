// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;

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

  public Robot() {
    
    motorSuperiorEsquerdoLider = new SparkMax(2, MotorType.kBrushless); //Estanciando variaveis com os parametros do motores.
    motorInferiorEsquerdoSeg = new SparkMax(3, MotorType.kBrushless); //Estanciando variaveis com os parametros do motores.
    motorSuperiorDireitoLider = new SparkMax(4, MotorType.kBrushless); //Estanciando variaveis com os parametros do motores.
    motorInferiorDireitoSeg = new SparkMax(5, MotorType.kBrushless); //Estanciando variaveis com os parametros do motores.

    SparkMaxConfig configGlobal = new SparkMaxConfig(); //Estaciando as configurações para motores
    SparkMaxConfig configMotorDireitoLider = new SparkMaxConfig();
    SparkMaxConfig configMotorEsquerdoSeg = new SparkMaxConfig();
    SparkMaxConfig configMotorDireitoSeg = new SparkMaxConfig();

    //Definindo Configurações
    configGlobal
        .smartCurrentLimit(30)
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

    //Parametrizzando as configurações
        motorSuperiorEsquerdoLider.configure(configGlobal, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motorInferiorEsquerdoSeg.configure(configMotorEsquerdoSeg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motorSuperiorDireitoLider.configure(configMotorDireitoLider, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motorInferiorDireitoSeg.configure(configMotorDireitoSeg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        logitech = new Joystick(0); //Estaciando o Joystick
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    
    double gatEsquerdo = -logitech.getRawAxis(2);
    double gatDireito = -logitech.getRawAxis(3);

    double analogEsquerdo = logitech.getRawAxis(0);

    double valorGat = gatDireito - gatEsquerdo;
    double ajusteAnalogEsquerdo = analogEsquerdo * 0.5;

    motorSuperiorEsquerdoLider.set(valorGat + ajusteAnalogEsquerdo);
    motorInferiorEsquerdoSeg.set(valorGat + ajusteAnalogEsquerdo);

    motorSuperiorDireitoLider.set(valorGat - ajusteAnalogEsquerdo);
    motorInferiorDireitoSeg.set(valorGat - ajusteAnalogEsquerdo);
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

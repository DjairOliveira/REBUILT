// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.io.File;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.*;
import frc.robot.subsystems.*;

public class RobotContainer {
    // velocidade de translação e rotação. definidas na Tuner Constants
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    private final PIDController headingPid = new PIDController(8.0, 0.0, 0.0);

    // private Rotation2d lockedHeading = new Rotation2d();
    private boolean isHeadingLocked = false;


    // // o robo ira dirigir de acordo com o campo.
    // private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
    //     .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
    //     .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    // @SuppressWarnings("unused")
    // private final Telemetry logger = new Telemetry(MaxSpeed);

    final CommandXboxController Cmdriver = new CommandXboxController(0);
    public XboxController driver = new XboxController(0);

    private SendableChooser<Command> autoChooser = new SendableChooser<>();

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private Hood mHood = new Hood(drivetrain);
    private Intake mIntake = new Intake();
    private Climber mClimber = new Climber(drivetrain);
    // public SubSystemSIM mSubSystemSIM = new SubSystemSIM();

    private int intakectn = 0;

    public double colisionProtect = 1;

    public RobotContainer() {

        configureBindings();

        // // Configuração do autochooser e definição das opções de auto
        // autoChooser = new SendableChooser<>();
        // autoChooser.setDefaultOption("TRENCH RIGHT", AutoBuilder.buildAuto("TrenchRight"));

        NamedCommands.registerCommand("updateOdometryCAM",
            Commands.runOnce(() -> drivetrain.odometryUpdateAutonomo(drivetrain.mt2Front)));

        NamedCommands.registerCommand("HOOD_HUB", new Hood(drivetrain));

        // new EventTrigger("CLIMBER_UP").onTrue(Commands.runOnce(() -> mClimber.setPosition(-350, 1)));
        // new EventTrigger("CLIMBER_PUSH").onTrue(Commands.runOnce(() -> mClimber.setPosition(0, 1)));

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto", autoChooser);

    }

    private void configureBindings() {

        Command driveNormal = drivetrain.driveFieldOrientedScaled(
            () -> -driver.getLeftY() * MaxSpeed * drivetrain.getColision(),
            () -> -driver.getLeftX() * MaxSpeed * drivetrain.getColision(),
            () -> -drivetrain.getOmegaCmd() * MaxAngularRate * drivetrain.getColision(), 1, 1);

        Command driveHood = drivetrain.driveFieldOrientedScaled(
            () -> -driver.getLeftY() * MaxSpeed * drivetrain.getColision(),
            () -> -driver.getLeftX() * MaxSpeed * drivetrain.getColision(),
            () -> -mHood.getOmega(), 1, 1);

        BooleanSupplier driverShooter = () -> driver.getLeftBumperButtonPressed();
        BooleanSupplier defaultMove = () -> driver.getLeftBumperButtonReleased() || Robot.elapsedTime < 1;
        Command driveMode = driveNormal;

        activateCommandOnCondition(defaultMove, driveMode = driveNormal);
        activateCommandOnCondition(driverShooter, driveMode = driveHood);

        drivetrain.setDefaultCommand(driveMode);
        Cmdriver.start().onTrue((Commands.runOnce(drivetrain::zeroGyro)));

        /********** INTAKE **************************/
        activateCommandOnCondition(() -> driver.getAButton(), new InstantCommand(() -> intakectn++));

        activateCommandOnCondition(() -> intakectn == 1, new SequentialCommandGroup(
            Commands.runOnce(() -> mIntake.setIntakeRPM(Robot.RPMShooter.getDouble(0))),
            Commands.runOnce(() -> drivetrain.m_SubSystemSIM.setIntakeVelocity(4)),
            Commands.runOnce(() -> drivetrain.m_SubSystemSIM.setSubIntake(20, 3, 0, 20)),
            Commands.runOnce(() -> drivetrain.m_SubSystemSIM.setSubClimber(drivetrain.getPose(), 100, 5, 0, 100))
            ));

        activateCommandOnCondition(() -> intakectn >= 2, new SequentialCommandGroup(
            Commands.runOnce(() -> mIntake.setIntakeRPM(0)),
            Commands.runOnce(() -> drivetrain.m_SubSystemSIM.setSubIntake(0, 3, 0, 20)),
            Commands.runOnce(() -> drivetrain.m_SubSystemSIM.setIntakeVelocity(0)),
            new InstantCommand(() -> intakectn = 0)));

        Cmdriver.povLeft().onTrue(Commands.runOnce(() -> drivetrain.m_SubSystemSIM.intakeVelocityCurrent(1)));
        Cmdriver.povLeft().onTrue(Commands.runOnce(() -> drivetrain.m_SubSystemSIM.shooterVelocityCurrent(3.2)));

        activateCommandOnCondition(() -> mHood.getarticulaAux(5, 2), new SequentialCommandGroup(
                new InstantCommand(() -> drivetrain.m_SubSystemSIM.setSubIntake(14, 1, 0, 20)),
                new InstantCommand(() -> intakectn = 0)));

        activateCommandOnCondition(() -> !mHood.getarticulaAux(2, 1), new InstantCommand(() -> intakectn = 1));

        // activateCommandOnCondition(() -> mHood.getarticulaAux(),
        //         new InstantCommand(() -> mSubSystemSIM.setSubClimber(0, 1, 0, 100)));

        /********** HOOD **************************/
        Cmdriver.b().onTrue(new SequentialCommandGroup(
            Commands.runOnce(() -> Hood.stopShooterSpeed()),
            Commands.runOnce(() -> Hood.stopIndexSpeed())
            // Commands.runOnce(() -> Intake.setBeltSpeed(0))
            ));

        Cmdriver.y().onTrue(Commands.runOnce(() -> mHood.setShooterRPM(10)));
        Cmdriver.y().onTrue(Commands.runOnce(() -> mHood.setIndexRPM(10)));
        // Cmdriver.y().onTrue(Commands.runOnce(() -> Intake.setBeltSpeed(0)));

        Cmdriver.leftBumper().whileTrue(mHood.repeatedly());
        Cmdriver.leftBumper().onFalse(Commands.runOnce(() -> mHood.end()));

        /* CLIMBER */
        // Cmdriver.povDown().onTrue(Commands.runOnce(() -> mClimber.setPosition(0,
        // 1))); //359 max alto
        // Cmdriver.povUp().onTrue(Commands.runOnce(() -> mClimber.setPosition(-350,
        // 1)));

        Cmdriver.povUp().onTrue(Commands.runOnce(() -> drivetrain.m_SubSystemSIM.setSubClimber(drivetrain.getPose(), 100, 5, 0, 100)));
        Cmdriver.povDown().onTrue(Commands.runOnce(() -> drivetrain.m_SubSystemSIM.setSubClimber(drivetrain.getPose(), 0, 5, 0, 100)));

    }

    public Command getAutonomousCommand() {
        // Primeiro o robô reseta o giroscópio de forma segura e DEPOIS lê a trajetória
        // que irá realizar
        return Commands.sequence(
                prepararInicioDePartidaCommand(),
                autoChooser.getSelected());
    }

    private void activateCommandOnCondition(BooleanSupplier condition, Command command) {
        new Trigger(condition).onTrue(command);
    }

    // Início quando no autônomo
    public Command prepararInicioDePartidaCommand() {
        return Commands.sequence(
                drivetrain.runOnce(() -> drivetrain.configAngleInit()),
                Commands.waitSeconds(0.1),
                drivetrain.runOnce(() -> {
                    headingPid.reset();
                    isHeadingLocked = false;
                })
                );
    }

    // Caso use no teleopInit
    public void prepararInicioDePartida() {
        prepararInicioDePartidaCommand().schedule();
    }

        // // // Método padrão do robô. Opera a todo momento que nada mais tiver sendo
        // // solicitado
        // drivetrain.setDefaultCommand(
        //     drivetrain.applyRequest(() -> {
        //         double triggerFactor = joystick.getRightTriggerAxis();

        //         double leftY = -joystick.getLeftY();
        //         double leftX = -joystick.getLeftX();
        //         double rightX = -joystick.getRightX();

        //         if (Math.abs(leftY) < 0.1) leftY = 0;
        //         if (Math.abs(leftX) < 0.1) leftX = 0;
        //         if (Math.abs(rightX) < 0.1) rightX = 0;

        //         double vxField = leftY * MaxSpeed * triggerFactor;
        //         double vyField = leftX * MaxSpeed * triggerFactor;
                
        //         double anguloAcumuladoBruto = drivetrain.getPigeon2().getYaw().getValueAsDouble();
        //         double anguloRealModulado = MathUtil.inputModulus(anguloAcumuladoBruto * Constants.FATOR_ESCALA_PIGEON, -180, 180);
                
        //         Pose2d poseAtual = drivetrain.getState().Pose;
        //         double rotOutput = 0.0;

        //         /* MIRAR NO HUB */
        //         if (joystick2.a().getAsBoolean()) {
        //             isAimLocked = false; 
        //             isHeadingLocked = false; 

        //             boolean isRed = edu.wpi.first.wpilibj.DriverStation.getAlliance().orElse(edu.wpi.first.wpilibj.DriverStation.Alliance.Blue) == edu.wpi.first.wpilibj.DriverStation.Alliance.Red;
        //             double alvoX = isRed ? Constants.HUB_X_Red : Constants.HUB_X_Blue;
        //             double alvoY = isRed ? Constants.HUB_Y_Red : Constants.HUB_Y_Blue;

        //             double diferencaX = alvoX - poseAtual.getX();
        //             double diferencaY = alvoY - poseAtual.getY();
                    
        //             Rotation2d anguloProHubContinuo = new Rotation2d(diferencaX, diferencaY).plus(Rotation2d.fromDegrees(180));

        //             double calcVisao = headingPid.calculate(Math.toRadians(anguloRealModulado), anguloProHubContinuo.getRadians());

        //             rotOutput = headingPid.atSetpoint() ? 0.0 : calcVisao;

        //             rotOutput = MathUtil.clamp(rotOutput, -Constants.LIMITE_ROTACAO, Constants.LIMITE_ROTACAO);

        //             // Com a mira para o HUB ativada, caso o piloto 1 segure R1 para atirar, será executado o comando abaixo, que visa deixar o robô 
        //             // na melhor distância possível do HUB, que definimos como 2,7 metros. 
        //             if (joystick.rightBumper().getAsBoolean()) {
        //                 // calcula a hipotenusa entre o robô e o HUB de acordo com as diferenças de x e y
        //                 double distanciaAtual = Math.hypot(diferencaX, diferencaY);
        //                 // Encontra a diferença entre a posição atual do robô e a distância alvo, que é 2,7 metros
        //                 double erroDistancia = distanciaAtual - 2.70; 

        //                 // Se o erro for maior que 0.05 metros, faz o robo andar ate a posição
        //                if (Math.abs(erroDistancia) > 0.05) {
        //                     double kP_Dist = 4.0; 
                            
        //                     // Inverte a direção dependendo da aliança devido a inversão do pigeon
        //                     double sinalPerspectiva = isRed ? -1.0 : 1.0;

        //                     //velocidade que o robo deve se mover para a posicao de 2.7m dependendo de onde esta
        //                     //Multiplicamos pela perspectiva ao inves de colocar sinal de negativo.
        //                     //por que assim, sempre que estiver definida a alianca, o valor muda automaticamente
        //                     double velMove = erroDistancia * kP_Dist * sinalPerspectiva;
        //                     //define valor maximo de velMove (-4 e 4)
        //                     velMove = MathUtil.clamp(velMove, -4, 4);

        //                     //define a velocidade maxima de translacao 
        //                     vxField = (diferencaX / distanciaAtual) * velMove;
        //                     vyField = (diferencaY / distanciaAtual) * velMove;

        //                     // Como isso é executado após a leitura do Joystick Esquerdo, ele ignora o Joystick Esquerdo e a translação segue esse comando
        //                 }
        //             }
        //         }

        //         // 2. ALINHAR PARA O 45° MAIS PRÓXIMO, facilitando a passagem por cima da rampa
        //         else if (joystick.y().getAsBoolean()) {
        //             //trava de mira desativada.
        //             isAimLocked = false; 
                    
        //             //manter a frente desativado para nao conflitar.
        //             isHeadingLocked = false; 
                    
        //             //passa os angulos diagonais possiveis para o robo estar virado a 45 graus pra array
        //             double[] angulosDiagonais = {45.0, 135.0, -45.0, -135.0};
        //             double alvo45 = 45.0;
        //             double menorDistancia = 360.0; //valor de 360 criado para ser substituido rapidamente
        //             // para que depois os proximos angulos possam ser comparados.
                    
        //             // pega a array de angulos diagonais e calcula qual e o mais proximo do angulo atual, definindo ele como o alvo no final do FOR
        //             for (double angulo : angulosDiagonais) {
        //                 //tenta encontrar o menor angulo entre todos na array, dentro de uma metrica de 180 a -180
        //                 // calculando a diferenca entre o angulo atual e um dos angulos da array, comparando cada um
        //                 double distancia = Math.abs(MathUtil.inputModulus(anguloRealModulado - angulo, -180, 180));
        //                 // se o valor na distancia calculada for menor que a distancia "recorde"
        //                 if (distancia < menorDistancia) {
        //                     //O antigo valor de distancia e trocado pelo menor valor
        //                     menorDistancia = distancia;
        //                     // transforma o valor de alvo45 no valor da array que apresentou o menor resultado
        //                     // de distancia durante a comparacao e guarda esse valor para passar pro pid
        //                     alvo45 = angulo;
        //                 }
        //             }

        //             //usa o objeto PID hehadingPid para calcular a forca necessaria para fazer o giro.
        //             //usa o angulo atual e o angulo que o robo precisa ir para fazer a conta.
        //             double calcAngulo = headingPid.calculate(
        //                 //transforma o valor em radianos. 
        //                 Math.toRadians(anguloRealModulado), 
        //                 Math.toRadians(alvo45)
        //             );
        //             //se estiver no ponto, nao manda forca pra rotacao
        //             //se nao usa o valor armazenado em calcAngulo
        //             rotOutput = headingPid.atSetpoint() ? 0.0 : calcAngulo;
        //             // limite de forca que pode ser passada para os motores.
        //             rotOutput = MathUtil.clamp(rotOutput, -Constants.LIMITE_ROTACAO, Constants.LIMITE_ROTACAO);
        //         }
                
        //         // 3. MIRAR PARA A TOWER, gira o robô para ficar de frente com a parede da tower
        //         else if (joystick2.y().getAsBoolean()) {
        //             //trava de mira desativada.
        //             isAimLocked = false; 
        //             //manter a frente desativado para nao conflitar.
        //             isHeadingLocked = false; 
                    
        //             // pega a alianca atual 
        //             boolean isRed = edu.wpi.first.wpilibj.DriverStation.getAlliance().orElse(edu.wpi.first.wpilibj.DriverStation.Alliance.Blue) == edu.wpi.first.wpilibj.DriverStation.Alliance.Red;
        //             // se a alianca for vermelha, o valor e 0 se for azul e 180, pode mudar devido a inversão do pigeon
        //             double alvoReto = isRed ? 0.0 : 180.0;

                   
        //             //usa o valor de pid para calcular a forca necessaria para fazer o giro.
        //             //usa o angulo atual e o angulo que precisa ir para fazer a conta.
        //             double calcAngulo = headingPid.calculate(
        //                 Math.toRadians(anguloRealModulado), 
        //                 Math.toRadians(alvoReto)
        //             );
                    
        //             //se estiver no ponto, nao manda forca pra rotacao
        //             //se nao usa o valor armazenado em calcAngulo
        //             rotOutput = headingPid.atSetpoint() ? 0.0 : calcAngulo;
        //             // limite de forca que pode ser passada para os motores.
        //             rotOutput = MathUtil.clamp(rotOutput, -Constants.LIMITE_ROTACAO, Constants.LIMITE_ROTACAO);
        //         }

        //         // 4. PILOTO ROTACIONANDO MANUALMENTE, executa quando nenhum outro comando de rotação foi ativado
        //         // se o valor horizontal do stick de rotacao (direito) for maior que 0
        //         else if (Math.abs(rightX) > 0) {
        //             //desliga a mira.
        //             isAimLocked = false; 
                    
        //             // se a funcao de manter a frente estiver ligada, desliga e reseta ele, para poder rotacionar o robô
        //             if (isHeadingLocked) {
        //                 isHeadingLocked = false;
        //                 headingPid.reset(); 
        //             }

        //             // define a saida de forca pros motores multiplicando o valor do stick
        //             // e a velocidade maxima angular.
        //             rotOutput = rightX * MaxAngularRate;
        //         } 

        //         // 5. NENHUM COMANDO DE GIRO ACIONADO - TRAVA DE MIRA ATIVADA
        //         else {
        //             // auto alinhamento desativada
        //             isAimLocked = false; 
                    
        //             // Voltamos a travar a mira caso tudo esteja solto.
        //             // Se a funcao de manter a frente estiver desligada ele resetará ela para a posição de giro atual e ativará a trava de mira.
        //             if (!isHeadingLocked) {
        //                 //pega a rotacao atual, o ângulo 
        //                 lockedHeading = Rotation2d.fromDegrees(anguloRealModulado);
        //                 //reseta o pid pra nao deixar vestigios
        //                 headingPid.reset(); 
        //                 //liga a funcao de manter a frente.
        //                 isHeadingLocked = true;
        //             }
                    
        //             // Gera um valor de PID a partir da diferença entre o atual ângulo do robô, que pode se alterar quando translada
        //             // e o ângulo definido pela trava de mira. Esse valor vista reduzir ao máximo a diferença entre os dois ângulos.
        //             double calcTrava = headingPid.calculate(
        //                 Math.toRadians(anguloRealModulado),
        //                 lockedHeading.getRadians()
        //             );

        //             // se estiver no setpoint, nao gira. Se nao, usa o valor criado pelo objeto PID.
        //             rotOutput = headingPid.atSetpoint() ? 0.0 : calcTrava;
        //             rotOutput = MathUtil.clamp(rotOutput, -Constants.LIMITE_ROTACAO, Constants.LIMITE_ROTACAO);
        //         }

        //         // Envia o ângulo  para Log
        //         anguloAtualEntry.setDouble(anguloRealModulado);
        //         // Envia o ângulo alvo atual do robô
        //         anguloAlvoEntry.setDouble(lockedHeading.getDegrees());

        //         return drive
        //         // define a velocidade translacional e rotacional de acordo com a ordem dada ao robo. 
        //             .withVelocityX(vxField)
        //             .withVelocityY(vyField)
        //             .withRotationalRate(rotOutput);
        //     })
        // );
        
        // // Função 1: Atualiza o 0° do robô, serve para quando ele inicia apontando para outra direção ou se deu drift.
        // joystick.start().onTrue(
        //     //realiza as seguintes funcoes
        //     Commands.sequence(
        //         // zera a odometria
        //         // Red = 180 graus (olhando para o fundo do campo Azul)
        //         // Blue = 0 graus (olhando para o fundo do campo Vermelho)
        //         drivetrain.runOnce(() -> drivetrain.configurarAnguloInicial()), 
        //         // espera 0.1 segundos
        //         Commands.waitSeconds(0.1), 
        //         // reseta o pid e desativa a funcao de manter a frente, para que quando passar pelo DefaultCommand, após passar por aqui,
        //         // defina um novo ângulo como sendo o ângulo alvo
        //         drivetrain.runOnce(() -> {
        //             headingPid.reset();      
        //             isHeadingLocked = false; 
        //         })
        //     )
        // );


}
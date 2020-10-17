within ;
package FM3217_2020 "Collection of models as created in FM3217"
  package Tutorial1
    model SimplePendulum "Model of a simple pendulum"
      import SI = Modelica.SIunits;
      constant SI.Acceleration g = 9.81 "Gravitational constant";
      parameter SI.Length L = 1 "Length of the pendulum";

     //Variables
      SI.Angle Theta( start=0.1, fixed=true) "Angle of the pendulum";
      /*
  Start of comment
  Some comment
  */
      SI.AngularVelocity ThetaDot;
    equation
      ThetaDot = der(Theta);
      der(ThetaDot) = -g/L*sin(Theta);
    end SimplePendulum;
  end Tutorial1;

  package Tutorial2
    model SimplePendulumTut1 "Model of a simple pendulum"
      import SI = Modelica.SIunits;
      constant SI.Acceleration g = 9.81 "Gravitational constant";
      parameter SI.Length L = 1 "Length of the pendulum";

     //Variables
      SI.Angle Theta( start=0.1, fixed=true) "Angle of the pendulum";
      /*
  Start of comment
  Some comment
  */
      SI.AngularVelocity ThetaDot;
    equation
      ThetaDot = der(Theta);
      der(ThetaDot) = -g/L*sin(Theta);
    end SimplePendulumTut1;

    model Motor
      Modelica.Electrical.Analog.Basic.Resistor Ra(R=0.5)
        annotation (Placement(transformation(extent={{-34,50},{-14,70}})));
      Modelica.Electrical.Analog.Basic.Inductor La(L=0.05)
        annotation (Placement(transformation(extent={{2,50},{22,70}})));
      Modelica.Electrical.Analog.Basic.EMF emf
        annotation (Placement(transformation(extent={{18,-10},{38,10}})));
      Modelica.Electrical.Analog.Basic.Ground ground
        annotation (Placement(transformation(extent={{-50,-42},{-30,-22}})));
      Modelica.Electrical.Analog.Sources.SignalVoltage signalVoltage
        annotation (Placement(transformation(
            extent={{-10,10},{10,-10}},
            rotation=-90,
            origin={-40,0})));
      Modelica.Mechanics.Rotational.Components.Inertia inertia(J=0.001)
        annotation (Placement(transformation(extent={{50,-10},{70,10}})));
      Modelica.Blocks.Interfaces.RealInput u
        annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
      Modelica.Mechanics.Rotational.Interfaces.Flange_b flange
        "Flange of right shaft"
        annotation (Placement(transformation(extent={{90,-10},{110,10}})));
    equation
      connect(La.n, emf.p)
        annotation (Line(points={{22,60},{28,60},{28,10}}, color={0,0,255}));
      connect(Ra.n, La.p)
        annotation (Line(points={{-14,60},{2,60}}, color={0,0,255}));
      connect(emf.flange, inertia.flange_a)
        annotation (Line(points={{38,0},{50,0}}, color={0,0,0}));
      connect(signalVoltage.n, ground.p)
        annotation (Line(points={{-40,-10},{-40,-22}}, color={0,0,255}));
      connect(emf.n, ground.p) annotation (Line(points={{28,-10},{28,-20},{-40,
              -20},{-40,-22}}, color={0,0,255}));
      connect(Ra.p, signalVoltage.p) annotation (Line(points={{-34,60},{-40,60},
              {-40,10}}, color={0,0,255}));
      connect(signalVoltage.v, u)
        annotation (Line(points={{-52,0},{-120,0}}, color={0,0,127}));
      connect(inertia.flange_b, flange)
        annotation (Line(points={{70,0},{100,0}}, color={0,0,0}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)),
        experiment(StopTime=100));
    end Motor;

    model Motordrive
      Motor motor
        annotation (Placement(transformation(extent={{0,24},{20,44}})));
      Modelica.Blocks.Math.Feedback positionerror
        annotation (Placement(transformation(extent={{-66,24},{-46,44}})));
      Modelica.Blocks.Sources.Step step(startTime=0.5)
        annotation (Placement(transformation(extent={{-94,24},{-74,44}})));
      Modelica.Blocks.Continuous.PID controller(
        k=10,
        Ti=2,
        Td=0.01)
        annotation (Placement(transformation(extent={{-32,24},{-12,44}})));
      Modelica.Mechanics.Rotational.Components.IdealGear gearbox(ratio=100)
        annotation (Placement(transformation(extent={{30,24},{50,44}})));
      Modelica.Mechanics.Rotational.Components.Inertia load(J=5)
        annotation (Placement(transformation(extent={{58,24},{78,44}})));
      Modelica.Mechanics.Rotational.Sensors.AngleSensor angleSensor annotation (
         Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=-90,
            origin={90,-6})));
    equation
      connect(step.y, positionerror.u1)
        annotation (Line(points={{-73,34},{-64,34}}, color={0,0,127}));
      connect(positionerror.y, controller.u)
        annotation (Line(points={{-47,34},{-34,34}}, color={0,0,127}));
      connect(controller.y, motor.u)
        annotation (Line(points={{-11,34},{-2,34}}, color={0,0,127}));
      connect(motor.flange, gearbox.flange_a)
        annotation (Line(points={{20,34},{30,34}}, color={0,0,0}));
      connect(gearbox.flange_b, load.flange_a)
        annotation (Line(points={{50,34},{58,34}}, color={0,0,0}));
      connect(load.flange_b, angleSensor.flange)
        annotation (Line(points={{78,34},{90,34},{90,4}}, color={0,0,0}));
      connect(angleSensor.phi, positionerror.u2) annotation (Line(points={{90,
              -17},{90,-38},{-56,-38},{-56,26}}, color={0,0,127}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end Motordrive;
  end Tutorial2;

  package Tutorial3
    package Components
      model Motor
        Modelica.Electrical.Analog.Basic.Resistor Ra(R=0.5)
          annotation (Placement(transformation(extent={{-34,50},{-14,70}})));
        Modelica.Electrical.Analog.Basic.Inductor La(L=0.05)
          annotation (Placement(transformation(extent={{2,50},{22,70}})));
        Modelica.Electrical.Analog.Basic.EMF emf
          annotation (Placement(transformation(extent={{18,-10},{38,10}})));
        Modelica.Electrical.Analog.Basic.Ground ground
          annotation (Placement(transformation(extent={{-50,-42},{-30,-22}})));
        Modelica.Electrical.Analog.Sources.SignalVoltage signalVoltage
          annotation (Placement(transformation(
              extent={{-10,10},{10,-10}},
              rotation=-90,
              origin={-40,0})));
        Modelica.Mechanics.Rotational.Components.Inertia inertia(J=0.001)
          annotation (Placement(transformation(extent={{50,-10},{70,10}})));
        Modelica.Blocks.Interfaces.RealInput u
          annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
        Modelica.Mechanics.Rotational.Interfaces.Flange_b flange
          "Flange of right shaft"
          annotation (Placement(transformation(extent={{90,-10},{110,10}})));
      equation
        connect(La.n, emf.p)
          annotation (Line(points={{22,60},{28,60},{28,10}}, color={0,0,255}));
        connect(Ra.n, La.p)
          annotation (Line(points={{-14,60},{2,60}}, color={0,0,255}));
        connect(emf.flange, inertia.flange_a)
          annotation (Line(points={{38,0},{50,0}}, color={0,0,0}));
        connect(signalVoltage.n, ground.p)
          annotation (Line(points={{-40,-10},{-40,-22}}, color={0,0,255}));
        connect(emf.n, ground.p) annotation (Line(points={{28,-10},{28,-20},{-40,
                -20},{-40,-22}}, color={0,0,255}));
        connect(Ra.p, signalVoltage.p) annotation (Line(points={{-34,60},{-40,60},
                {-40,10}}, color={0,0,255}));
        connect(signalVoltage.v, u)
          annotation (Line(points={{-52,0},{-120,0}}, color={0,0,127}));
        connect(inertia.flange_b, flange)
          annotation (Line(points={{70,0},{100,0}}, color={0,0,0}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
              coordinateSystem(preserveAspectRatio=false)),
          experiment(StopTime=100));
      end Motor;

      model DCMachine

        parameter Modelica.SIunits.Resistance R=0.5
          "Resistance of the armature" annotation (Dialog(tab="Electrical"));

        Modelica.Electrical.Analog.Basic.Resistor Ra(R=R)
          annotation (Placement(transformation(extent={{-34,50},{-14,70}})));
        Modelica.Electrical.Analog.Basic.Inductor La(L=L)
          annotation (Placement(transformation(extent={{2,50},{22,70}})));
        Modelica.Electrical.Analog.Basic.EMF emf
          annotation (Placement(transformation(extent={{18,-10},{38,10}})));
        Modelica.Electrical.Analog.Basic.Ground ground
          annotation (Placement(transformation(extent={{-52,-60},{-32,-40}})));
        Modelica.Mechanics.Rotational.Components.Inertia inertia(J=0.001)
          annotation (Placement(transformation(extent={{50,-10},{70,10}})));
        Modelica.Mechanics.Rotational.Interfaces.Flange_b flange
          "Flange of right shaft"
          annotation (Placement(transformation(extent={{90,-10},{110,10}})));
        Modelica.Electrical.Analog.Interfaces.PositivePin p "Positive electrical pin"
          annotation (Placement(transformation(extent={{-114,44},{-84,74}})));
        Modelica.Electrical.Analog.Interfaces.Pin n annotation (Placement(
              transformation(extent={{-114,-66},{-86,-38}}), iconTransformation(
                extent={{-114,-66},{-86,-38}})));
        parameter Modelica.SIunits.Inductance L=0.1 "Inductance of DC machine"
          annotation (Dialog(tab="Electrical"));
        parameter Modelica.SIunits.Inertia J=0.001
          annotation (Dialog(tab="Mechanical"));
      equation
        connect(La.n, emf.p)
          annotation (Line(points={{22,60},{28,60},{28,10}}, color={0,0,255}));
        connect(Ra.n, La.p)
          annotation (Line(points={{-14,60},{2,60}}, color={0,0,255}));
        connect(emf.flange, inertia.flange_a)
          annotation (Line(points={{38,0},{50,0}}, color={0,0,0}));
        connect(emf.n, ground.p) annotation (Line(points={{28,-10},{28,-40},{-42,-40}},
                                 color={0,0,255}));
        connect(inertia.flange_b, flange)
          annotation (Line(points={{70,0},{100,0}}, color={0,0,0}));
        connect(Ra.p, p) annotation (Line(points={{-34,60},{-68,60},{-68,59},{-99,59}},
              color={0,0,255}));
        connect(ground.p, n) annotation (Line(points={{-42,-40},{-72,-40},{-72,-52},{-100,
                -52}}, color={0,0,255}));
        connect(n, n)
          annotation (Line(points={{-100,-52},{-100,-52}}, color={0,0,255}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
                Bitmap(extent={{100,-60},{-100,80}}, fileName="modelica://FM3217_2020/Resources/Images/dc-motor.jpg")}),
                                                                       Diagram(
              coordinateSystem(preserveAspectRatio=false)),
          experiment(StopTime=100),
          Documentation(info="<html>
<h4>This is a model of a simple of DC machine</h4>
</html>"));
      end DCMachine;

      model RLoad "Resistive load"
        Modelica.Electrical.Analog.Basic.Resistor resistor(R=R_load)
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=-90,
              origin={-18,0})));
        Modelica.Electrical.Analog.Interfaces.PositivePin p1
          "Positive electrical pin"
          annotation (Placement(transformation(extent={{-28,90},{-8,110}})));
        Modelica.Electrical.Analog.Interfaces.NegativePin n1
                      "Negative electrical pin"
          annotation (Placement(transformation(extent={{-28,-110},{-8,-90}})));
        parameter Modelica.SIunits.Resistance R_load=0.5
          "Resistive value of resistor";
      equation
        connect(resistor.p, p1)
          annotation (Line(points={{-18,10},{-18,100}}, color={0,0,255}));
        connect(resistor.n, n1)
          annotation (Line(points={{-18,-10},{-18,-100}}, color={0,0,255}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
              coordinateSystem(preserveAspectRatio=false)));
      end RLoad;

      model RLLoad
        extends RLoad;
        Modelica.Electrical.Analog.Basic.Inductor inductor(L=L_load)
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=-90,
              origin={-40,0})));
        parameter Modelica.SIunits.Inductance L_load=0.1 "Load inductance";
      equation
        connect(inductor.p, p1) annotation (Line(points={{-40,10},{-40,34},{-18,
                34},{-18,100}}, color={0,0,255}));
        connect(inductor.n, n1) annotation (Line(points={{-40,-10},{-40,-38},{
                -18,-38},{-18,-100}}, color={0,0,255}));
      end RLLoad;

      model RLCLoad
        extends RLLoad;
        Modelica.Electrical.Analog.Basic.Capacitor capacitor(C=C_load)
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=-90,
              origin={-60,0})));
        parameter Modelica.SIunits.Capacitance C_load=0.001 "Load capcitance";
      equation
        connect(capacitor.p, p1) annotation (Line(points={{-60,10},{-60,34},{
                -18,34},{-18,100}}, color={0,0,255}));
        connect(capacitor.n, n1) annotation (Line(points={{-60,-10},{-60,-38},{
                -18,-38},{-18,-100}}, color={0,0,255}));
      end RLCLoad;

      model Turbine
        Modelica.Mechanics.Rotational.Components.Inertia inertia(J=J_t)
          annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
        Modelica.Mechanics.Rotational.Sources.ConstantTorque constantTorque(
            tau_constant=T_t)
          annotation (Placement(transformation(extent={{62,-10},{42,10}})));
        Modelica.Mechanics.Rotational.Interfaces.Flange_a flange_a annotation (
            Placement(transformation(rotation=0, extent={{-110,-10},{-90,10}})));
        parameter Modelica.SIunits.Inertia J_t=2 "Turbine inertia";
        parameter Modelica.SIunits.AngularMomentum T_t=10 "Turbine torque";
      equation
        connect(constantTorque.flange, inertia.flange_b)
          annotation (Line(points={{42,0},{10,0}}, color={0,0,0}));
        connect(flange_a, inertia.flange_a)
          annotation (Line(points={{-100,0},{-10,0}}, color={0,0,0}));
        annotation (Icon(graphics={Bitmap(extent={{-100,-72},{114,80}},
                  fileName=
                    "modelica://FM3217_2020/Resources/Images/Turbine.png")}));
      end Turbine;
    end Components;

    package Tests
      model Motordrive
        Tutorial2.Motor motor
          annotation (Placement(transformation(extent={{0,24},{20,44}})));
        Modelica.Blocks.Math.Feedback positionerror
          annotation (Placement(transformation(extent={{-66,24},{-46,44}})));
        Modelica.Blocks.Sources.Step step(startTime=0.5)
          annotation (Placement(transformation(extent={{-94,24},{-74,44}})));
        Modelica.Blocks.Continuous.PID controller(
          k=10,
          Ti=2,
          Td=0.01)
          annotation (Placement(transformation(extent={{-32,24},{-12,44}})));
        Modelica.Mechanics.Rotational.Components.IdealGear gearbox(ratio=100)
          annotation (Placement(transformation(extent={{30,24},{50,44}})));
        Modelica.Mechanics.Rotational.Components.Inertia load(J=5)
          annotation (Placement(transformation(extent={{58,24},{78,44}})));
        Modelica.Mechanics.Rotational.Sensors.AngleSensor angleSensor annotation (
           Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=-90,
              origin={90,-6})));
      equation
        connect(step.y, positionerror.u1)
          annotation (Line(points={{-73,34},{-64,34}}, color={0,0,127}));
        connect(positionerror.y, controller.u)
          annotation (Line(points={{-47,34},{-34,34}}, color={0,0,127}));
        connect(controller.y, motor.u)
          annotation (Line(points={{-11,34},{-2,34}}, color={0,0,127}));
        connect(motor.flange, gearbox.flange_a)
          annotation (Line(points={{20,34},{30,34}}, color={0,0,0}));
        connect(gearbox.flange_b, load.flange_a)
          annotation (Line(points={{50,34},{58,34}}, color={0,0,0}));
        connect(load.flange_b, angleSensor.flange)
          annotation (Line(points={{78,34},{90,34},{90,4}}, color={0,0,0}));
        connect(angleSensor.phi, positionerror.u2) annotation (Line(points={{90,
                -17},{90,-38},{-56,-38},{-56,26}}, color={0,0,127}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
              coordinateSystem(preserveAspectRatio=false)));
      end Motordrive;

      model DCMachineTest
        Components.DCMachine dCMachine(R=1)
          annotation (Placement(transformation(extent={{12,8},{32,28}})));
        Modelica.Electrical.Analog.Sources.SignalVoltage signalVoltage
          annotation (Placement(transformation(
              extent={{-10,10},{10,-10}},
              rotation=-90,
              origin={-40,16})));
        Modelica.Blocks.Sources.Step step(startTime=0.5)
          annotation (Placement(transformation(extent={{-90,6},{-70,26}})));
        Modelica.Mechanics.Rotational.Components.Inertia inertia
          annotation (Placement(transformation(extent={{56,8},{76,28}})));
      equation
        connect(signalVoltage.n, dCMachine.n) annotation (Line(points={{-40,6},
                {-14,6},{-14,12.8},{12,12.8}}, color={0,0,255}));
        connect(signalVoltage.p, dCMachine.p) annotation (Line(points={{-40,26},
                {-14,26},{-14,23.9},{12.1,23.9}}, color={0,0,255}));
        connect(signalVoltage.v, step.y)
          annotation (Line(points={{-52,16},{-69,16}}, color={0,0,127}));
        connect(dCMachine.flange, inertia.flange_a)
          annotation (Line(points={{32,18},{56,18}}, color={0,0,0}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
              coordinateSystem(preserveAspectRatio=false)));
      end DCMachineTest;

      model DCGeneratorTest
        Components.DCMachine dCMachine(R=1)
          annotation (Placement(transformation(extent={{-44,-10},{-24,10}})));
        Components.RLCLoad rLCLoad
          annotation (Placement(transformation(extent={{-88,-14},{-68,6}})));
        Components.Turbine turbine annotation (Placement(transformation(
                rotation=0, extent={{22,-10},{42,10}})));
      equation
        connect(dCMachine.flange, turbine.flange_a)
          annotation (Line(points={{-24,0},{22,0}}, color={0,0,0}));
        connect(rLCLoad.p1, dCMachine.p) annotation (Line(points={{-79.8,6},{
                -80,6},{-80,20},{-43.9,20},{-43.9,5.9}}, color={0,0,255}));
        connect(rLCLoad.n1, dCMachine.n) annotation (Line(points={{-79.8,-14},{
                -80,-14},{-80,-20},{-44,-20},{-44,-5.2}}, color={0,0,255}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
              coordinateSystem(preserveAspectRatio=false)));
      end DCGeneratorTest;
    end Tests;
  end Tutorial3;

  package Tutorial4
    model ElectricKettle
      Modelica.Electrical.Analog.Basic.Resistor resistor(R=230^2/2000,
          useHeatPort=true) annotation (Placement(transformation(
            extent={{-10,10},{10,-10}},
            rotation=-90,
            origin={-8,0})));
      Modelica.Electrical.Analog.Basic.Ground ground
        annotation (Placement(transformation(extent={{-90,-80},{-70,-60}})));
      Modelica.Electrical.Analog.Sources.SineVoltage sineVoltage(V=sqrt(2)*230,
          freqHz=50) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=-90,
            origin={-80,-2})));
      Modelica.Electrical.Analog.Sensors.PowerSensor powerSensor
        annotation (Placement(transformation(extent={{-40,10},{-20,30}})));
      Modelica.Blocks.Math.Mean mean(f=50) annotation (Placement(transformation(
            extent={{-4,-4},{4,4}},
            rotation=-90,
            origin={-40,-4})));
      Modelica.Thermal.HeatTransfer.Components.HeatCapacitor water(C=4.18e3*1.7,
          T(start=283.15, fixed=true))
        annotation (Placement(transformation(extent={{10,6},{30,26}})));
      Modelica.Thermal.HeatTransfer.Celsius.TemperatureSensor temperatureSensor
        annotation (Placement(transformation(extent={{40,-10},{60,10}})));
      Modelica.Thermal.HeatTransfer.Components.ThermalConductor KettleWall(G=5)
        annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=-90,
            origin={20,-30})));
      Modelica.Thermal.HeatTransfer.Sources.FixedTemperature roomTemperature(T=
            294.15)
        annotation (Placement(transformation(extent={{70,-86},{50,-66}})));
      Modelica.Blocks.Logical.OnOffController onOffController(bandwidth=3)
        annotation (Placement(transformation(extent={{74,-4},{94,16}})));
      Modelica.Blocks.Sources.Constant const(k=95)
        annotation (Placement(transformation(extent={{22,32},{42,52}})));
      Modelica.Electrical.Analog.Ideal.IdealClosingSwitch switch1
        annotation (Placement(transformation(extent={{-70,10},{-50,30}})));
    equation
      connect(ground.p, resistor.n) annotation (Line(points={{-80,-60},{-80,-20},
              {-8,-20},{-8,-10}}, color={0,0,255}));
      connect(sineVoltage.n, resistor.n) annotation (Line(points={{-80,-12},{
              -80,-20},{-8,-20},{-8,-10}}, color={0,0,255}));
      connect(powerSensor.nc, resistor.p)
        annotation (Line(points={{-20,20},{-8,20},{-8,10}}, color={0,0,255}));
      connect(powerSensor.nv, resistor.n) annotation (Line(points={{-30,10},{
              -30,-20},{-8,-20},{-8,-10}}, color={0,0,255}));
      connect(powerSensor.pv, resistor.p) annotation (Line(points={{-30,30},{
              -30,40},{-8,40},{-8,10}}, color={0,0,255}));
      connect(powerSensor.power, mean.u)
        annotation (Line(points={{-40,9},{-40,0.8}}, color={0,0,127}));
      connect(resistor.heatPort, water.port)
        annotation (Line(points={{2,0},{20,0},{20,6}}, color={191,0,0}));
      connect(resistor.heatPort, temperatureSensor.port)
        annotation (Line(points={{2,0},{40,0}}, color={191,0,0}));
      connect(KettleWall.port_a, water.port)
        annotation (Line(points={{20,-20},{20,6}}, color={191,0,0}));
      connect(roomTemperature.port, KettleWall.port_b) annotation (Line(points=
              {{50,-76},{20,-76},{20,-40}}, color={191,0,0}));
      connect(temperatureSensor.T, onOffController.u)
        annotation (Line(points={{60,0},{72,0}}, color={0,0,127}));
      connect(const.y, onOffController.reference) annotation (Line(points={{43,
              42},{56,42},{56,12},{72,12}}, color={0,0,127}));
      connect(powerSensor.pc, switch1.n)
        annotation (Line(points={{-40,20},{-50,20}}, color={0,0,255}));
      connect(sineVoltage.p, switch1.p)
        annotation (Line(points={{-80,8},{-80,20},{-70,20}}, color={0,0,255}));
      connect(onOffController.y, switch1.control) annotation (Line(points={{95,
              6},{96,6},{96,74},{-60,74},{-60,32}}, color={255,0,255}));
      annotation (
        Icon(coordinateSystem(preserveAspectRatio=false)),
        Diagram(coordinateSystem(preserveAspectRatio=false)),
        Documentation(info="<html>
<p>Electric Kettle</p>
<ul>
<li>Volume of 1.7 litre</li>
<li>230 V main supply</li>
<li>Power = 2000 Watt</li>
</ul>
<p><br>Question: What should the resistance of the heating resisor be?</p>
<p><br>Power = Voltage * Current</p>
<p>Resitance = Voltage / Current</p>
<p><br>This leads to: </p>
<p>For a 2000 Watt consuming resisor the resistancve should be:</p>
<p>R = V/I = V /P /V= V^2/P</p>
<p><br><h4>Heat capacity of Water</h4></p>
<p>1 calory = heat energy to heat up one gram of water by 1 Kelvin</p>
<p>1calory = 4,18 J/(g K)</p>
</html>"),
        experiment(StopTime=600, __Dymola_NumberOfIntervals=5000));
    end ElectricKettle;
  end Tutorial4;

  package Tutorial5
    model ConnectingPipes
      HydroPower.HydroSystems.Pipe pipe(L=100, ZL=90)
        annotation (Placement(transformation(extent={{-10,56},{10,76}})));
      inner HydroPower.System_HPL system_HPL(steadyState=true,
          constantTemperature=true)
        annotation (Placement(transformation(extent={{-100,80},{-80,100}})));
      HydroPower.SinksAndSources.Fixed_pT source(paraOption=false)
        annotation (Placement(transformation(extent={{-56,56},{-36,76}})));
      HydroPower.SinksAndSources.Fixed_pT sink(paraOption=false)
        annotation (Placement(transformation(extent={{60,56},{40,76}})));
    equation
      connect(pipe.a, source.b)
        annotation (Line(points={{-11,66},{-35,66}}, color={0,0,255}));
      connect(pipe.b, sink.b)
        annotation (Line(points={{11,66},{39,66}}, color={0,0,255}));
      annotation (
        Icon(coordinateSystem(preserveAspectRatio=false)),
        Diagram(coordinateSystem(preserveAspectRatio=false)),
        experiment(
          StopTime=600,
          Tolerance=1e-05,
          __Dymola_Algorithm="Radau"));
    end ConnectingPipes;

    model PipeWithValve
      extends ConnectingPipes;
      HydroPower.HydroSystems.PipeValve pipeValve1(
        m_dot_nom=123500,
        dp_nom=900000,
        ZL=90) annotation (Placement(transformation(extent={{-10,10},{10,30}})));
      HydroPower.SinksAndSources.Fixed_pT source1(paraOption=false)
        annotation (Placement(transformation(extent={{-56,10},{-36,30}})));
      HydroPower.SinksAndSources.Fixed_pT sink1(paraOption=false)
        annotation (Placement(transformation(extent={{60,10},{40,30}})));
      Modelica.Blocks.Sources.Ramp ramp(
        height=0.9,
        duration=100,
        offset=0.1,
        startTime=100)
        annotation (Placement(transformation(extent={{-86,30},{-66,50}})));
    equation
      connect(pipeValve1.a, source1.b)
        annotation (Line(points={{-11,20},{-35,20}}, color={0,0,255}));
      connect(pipeValve1.b, sink1.b)
        annotation (Line(points={{11,20},{39,20}}, color={0,0,255}));
      connect(ramp.y, pipeValve1.ValveCtrl)
        annotation (Line(points={{-65,40},{0,40},{0,31}}, color={0,0,127}));
      annotation (Documentation(info="<html>
<p><br>P = <img src=\"modelica://FM3217_2020/Resources/Images/equations/equation-NlsdyTwU.png\" alt=\"rho
\"/> g Q H</p>
<p>P = 100 MW</p>
<p>H = 90 m</p>
<p>Q = P/(<img src=\"modelica://FM3217_2020/Resources/Images/equations/equation-z53iZlUM.png\" alt=\"rho\"/>*g*H) = 100e6 / (1e3 *9.81*90)</p>
</html>"));
    end PipeWithValve;

    model SimpleWaterWay
      extends PipeWithValve(ramp(
          height=0.99,
          duration=10,
          offset=0.01));
      HydroPower.SinksAndSources.Fixed_pT source2(paraOption=false)
        annotation (Placement(transformation(extent={{-100,-30},{-80,-10}})));
      HydroPower.SinksAndSources.Fixed_pT sink2(paraOption=false)
        annotation (Placement(transformation(extent={{92,-30},{72,-10}})));
      HydroPower.HydroSystems.Pipe pipe2(
        horizontalIcon=true,
        L=10000,
        ZL=100,
        ZR=90)
        annotation (Placement(transformation(extent={{-58,-30},{-38,-10}})));
      HydroPower.HydroSystems.PipeValve pipeValve2(
        m_dot_nom=110e3,
        dp_nom=900000,
        ZL=90,
        ZR=0) annotation (Placement(transformation(extent={{8,-30},{28,-10}})));
      HydroPower.HydroSystems.HydroComponents.Containers.ClosedVolume
        closedVolume
        annotation (Placement(transformation(extent={{-30,-30},{-10,-10}})));
    equation
      connect(source2.b, pipe2.a)
        annotation (Line(points={{-79,-20},{-59,-20}}, color={0,0,255}));
      connect(pipeValve2.ValveCtrl, pipeValve1.ValveCtrl) annotation (Line(
            points={{18,-9},{18,40},{0,40},{0,31}}, color={0,0,127}));
      connect(pipeValve2.b, sink2.b)
        annotation (Line(points={{29,-20},{71,-20}}, color={0,0,255}));
      connect(pipe2.b, closedVolume.a)
        annotation (Line(points={{-37,-20},{-30,-20}}, color={0,0,255}));
      connect(closedVolume.b, pipeValve2.a)
        annotation (Line(points={{-10,-20},{7,-20}}, color={0,0,255}));
      annotation (experiment(
          StopTime=600,
          __Dymola_NumberOfIntervals=5000,
          Tolerance=1e-05,
          __Dymola_Algorithm="Radau"));
    end SimpleWaterWay;

    model WaterWaySurgeShaft
      extends SimpleWaterWay(ramp(height=-0.9, offset=1));
      HydroPower.HydroSystems.SurgeTank surgeTank3(
        D=100,
        deltZ=15,
        Vol=1000)
        annotation (Placement(transformation(extent={{-28,-62},{-8,-42}})));
      HydroPower.SinksAndSources.Fixed_pT source3(paraOption=false)
        annotation (Placement(transformation(extent={{-98,-62},{-78,-42}})));
      HydroPower.HydroSystems.Pipe pipe3(
        horizontalIcon=true,
        L=10000,
        ZL=100,
        ZR=90)
        annotation (Placement(transformation(extent={{-60,-62},{-40,-42}})));
      HydroPower.HydroSystems.PipeValve pipeValve3(
        m_dot_nom=110e3,
        dp_nom=900000,
        ZL=90,
        ZR=0) annotation (Placement(transformation(extent={{8,-62},{28,-42}})));
      HydroPower.SinksAndSources.Fixed_pT sink3(paraOption=false)
        annotation (Placement(transformation(extent={{92,-62},{72,-42}})));
    equation
      connect(source3.b, pipe3.a)
        annotation (Line(points={{-77,-52},{-61,-52}}, color={0,0,255}));
      connect(pipe3.b, surgeTank3.a)
        annotation (Line(points={{-39,-52},{-29,-52}}, color={0,0,255}));
      connect(surgeTank3.b, pipeValve3.a)
        annotation (Line(points={{-7,-52},{7,-52}}, color={0,0,255}));
      connect(pipeValve3.ValveCtrl, pipeValve1.ValveCtrl) annotation (Line(
            points={{18,-41},{18,-34},{44,-34},{44,-4},{18,-4},{18,40},{0,40},{
              0,31}}, color={0,0,127}));
      connect(sink3.b, pipeValve3.b)
        annotation (Line(points={{71,-52},{29,-52}}, color={0,0,255}));
      annotation (experiment(
          StopTime=600,
          __Dymola_NumberOfIntervals=5000,
          Tolerance=1e-05,
          __Dymola_Algorithm="Radau"));
    end WaterWaySurgeShaft;
  end Tutorial5;

  package Tutorial6
    model ReservoirBaseModel
      inner HydroPower.System_HPL system_HPL(steadyState=true,
          constantTemperature=true)
        annotation (Placement(transformation(extent={{-100,80},{-80,100}})));
      HydroPower.HydroSystems.Reservoir headwater
        annotation (Placement(transformation(extent={{-70,40},{-50,60}})));
      HydroPower.HydroSystems.Reservoir tailwater
        annotation (Placement(transformation(extent={{46,0},{66,20}})));
      HydroPower.HydroSystems.Pipe conduit(horizontalIcon=true, L=1000)
        annotation (Placement(transformation(extent={{-42,34},{-22,54}})));
    equation
      connect(headwater.a2_pipe, conduit.a)
        annotation (Line(points={{-49,44},{-43,44}}, color={0,0,255}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end ReservoirBaseModel;

    model TwoReservoirs
      extends Modelica.Icons.Example;
      extends ReservoirBaseModel(conduit(ZL=100));
    equation
      connect(conduit.b, tailwater.a1_pipe) annotation (Line(points={{-21,44},{
              20,44},{20,4},{59,4}}, color={0,0,255}));
      annotation (experiment(StopTime=600, __Dymola_Algorithm="Radau"));
    end TwoReservoirs;

    model TwoReservoirsWithSource
      extends Modelica.Icons.Example;
      extends ReservoirBaseModel(conduit(ZL=100));
      HydroPower.SinksAndSources.Fixed_HT constantWaterHead(
        paraOption=false,
        H_const=75,
        Hmax=100,
        depth=50)
        annotation (Placement(transformation(extent={{-98,46},{-78,66}})));
      HydroPower.SinksAndSources.Fixed_HT constantTailWater(
        paraOption=false,
        H_const=75,
        Hmax=100,
        depth=50)
        annotation (Placement(transformation(extent={{98,6},{78,26}})));
    equation
      connect(conduit.b, tailwater.a1_pipe) annotation (Line(points={{-21,44},{
              18,44},{18,4},{45,4}}, color={0,0,255}));
      connect(constantWaterHead.b, headwater.a1_open)
        annotation (Line(points={{-77,56},{-71,56}}, color={0,0,255}));
      connect(tailwater.a2_open, constantTailWater.b)
        annotation (Line(points={{67,16},{77,16}}, color={0,0,255}));
    end TwoReservoirsWithSource;

    model WaterWayRes
      extends Modelica.Icons.Example;
      extends ReservoirBaseModel(conduit(
          L=10000,
          ZL=100,
          ZR=90));
      HydroPower.SinksAndSources.Fixed_HT constantWaterHead(
        paraOption=false,
        H_const=75,
        Hmax=100,
        depth=50)
        annotation (Placement(transformation(extent={{-98,46},{-78,66}})));
      HydroPower.SinksAndSources.Fixed_HT constantTailWater(
        paraOption=false,
        H_const=75,
        Hmax=100,
        depth=50)
        annotation (Placement(transformation(extent={{96,6},{76,26}})));
      HydroPower.HydroSystems.SurgeTank surgeTank(D=30, deltZ=100)
        annotation (Placement(transformation(extent={{-10,34},{10,54}})));
      HydroPower.HydroSystems.PipeValve pressureshaft(ZL=90)
        annotation (Placement(transformation(extent={{16,-6},{36,14}})));
      Modelica.Blocks.Sources.Ramp ramp(duration=10, startTime=100)
        annotation (Placement(transformation(extent={{-12,64},{8,84}})));
    equation
      connect(headwater.a1_open, constantWaterHead.b)
        annotation (Line(points={{-71,56},{-77,56}}, color={0,0,255}));
      connect(tailwater.a2_open, constantTailWater.b)
        annotation (Line(points={{67,16},{75,16}}, color={0,0,255}));
      connect(conduit.b, surgeTank.a)
        annotation (Line(points={{-21,44},{-11,44}}, color={0,0,255}));
      connect(surgeTank.b, pressureshaft.a) annotation (Line(points={{11,44},{
              14,44},{14,4},{15,4}}, color={0,0,255}));
      connect(tailwater.a1_pipe, pressureshaft.b)
        annotation (Line(points={{45,4},{37,4}}, color={0,0,255}));
      connect(ramp.y, pressureshaft.ValveCtrl)
        annotation (Line(points={{9,74},{26,74},{26,15}}, color={0,0,127}));
      annotation (experiment(
          StopTime=600,
          __Dymola_NumberOfIntervals=5000,
          __Dymola_Algorithm="Radau"));
    end WaterWayRes;

    model WaterWatResClosingValve
      extends WaterWayRes(ramp(
          height=-1,
          duration=1,
          offset=1), system_HPL(Q_start=31.276));
      annotation (experiment(
          StopTime=600,
          __Dymola_NumberOfIntervals=5000,
          __Dymola_Algorithm="Radau"));
    end WaterWatResClosingValve;

    model SundsbarmWaterway
      extends Modelica.Icons.Example;
      inner HydroPower.System_HPL system_HPL(
        Q_start=24,
        steadyState=true,
        constantTemperature=true)
        annotation (Placement(transformation(extent={{-100,80},{-80,100}})));
      HydroPower.HydroSystems.Reservoir headwater(
        Hmax=ones(headwater.n)*(564 + 48 + 5),
        depth=ones(headwater.n)*(48 + 5),
        H_start=ones(headwater.n)*(564 + 48),
        steadyState=true)
        annotation (Placement(transformation(extent={{-96,16},{-76,36}})));
      HydroPower.HydroSystems.Reservoir tailwater(
        Hmax=ones(tailwater.n)*(110 + 5 + 3),
        depth=ones(tailwater.n)*(5 + 3),
        H_start=ones(tailwater.n)*(110 + 5))
        annotation (Placement(transformation(extent={{74,-8},{94,12}})));
      HydroPower.HydroSystems.Pipe conduit(
        endD={5.8,5.8},
        L=6600,
        ZL=564,
        ZR=541.5,
        horizontalIcon=true)
        annotation (Placement(transformation(extent={{-70,10},{-50,30}})));
      HydroPower.SinksAndSources.Fixed_HT constantWaterHead(
        paraOption=false,
        H_const=564 + 48,
        Hmax=564 + 48 + 5,
        depth=48 + 5)
        annotation (Placement(transformation(extent={{-74,50},{-94,70}})));
      HydroPower.SinksAndSources.Fixed_HT constantTailWater(
        paraOption=false,
        H_const=115,
        Hmax=118,
        depth=8)
        annotation (Placement(transformation(extent={{70,20},{90,40}})));
      HydroPower.HydroSystems.SurgeTank surgeTank(
        D=3.6,
        deltZ=150,
        H2L=0.87,
        Vol=100)
        annotation (Placement(transformation(extent={{-40,10},{-20,30}})));
      HydroPower.HydroSystems.PipeValve pressureshaft(
        endD={3,3},
        m_dot_nom=24e3,
        dp_nom=4893000,
        L=724,
        ZL=541.5,
        ZR=112.5)
        annotation (Placement(transformation(extent={{-10,4},{10,24}})));
      Modelica.Blocks.Sources.Ramp ramp(
        height=-0.9,
        duration=500,
        offset=1,
        startTime=100)
        annotation (Placement(transformation(extent={{-40,40},{-20,60}})));
      HydroPower.HydroSystems.Pipe tailRace(
        endD={5.8,5.8},
        L=600,
        ZL=110.5,
        ZR=110,
        horizontalIcon=true)
        annotation (Placement(transformation(extent={{46,-14},{66,6}})));
      HydroPower.HydroSystems.HydroComponents.Containers.ClosedVolume
        turbineHourse(D=5.8, L=2)
        annotation (Placement(transformation(extent={{24,-2},{36,10}})));
    equation
      connect(headwater.a2_pipe, conduit.a)
        annotation (Line(points={{-75,20},{-71,20}}, color={0,0,255}));
      connect(headwater.a1_open, constantWaterHead.b) annotation (Line(points={
              {-97,32},{-100,32},{-100,60},{-95,60}}, color={0,0,255}));
      connect(tailwater.a2_open, constantTailWater.b) annotation (Line(points={
              {95,8},{96,8},{96,30},{91,30}}, color={0,0,255}));
      connect(conduit.b, surgeTank.a)
        annotation (Line(points={{-49,20},{-41,20}}, color={0,0,255}));
      connect(surgeTank.b, pressureshaft.a) annotation (Line(points={{-19,20},{
              -14,20},{-14,14},{-11,14}}, color={0,0,255}));
      connect(ramp.y, pressureshaft.ValveCtrl)
        annotation (Line(points={{-19,50},{0,50},{0,25}}, color={0,0,127}));
      connect(tailwater.a1_pipe, tailRace.b)
        annotation (Line(points={{73,-4},{67,-4}}, color={0,0,255}));
      connect(tailRace.a, turbineHourse.b) annotation (Line(points={{45,-4},{40,
              -4},{40,4},{36,4}}, color={0,0,255}));
      connect(pressureshaft.b, turbineHourse.a) annotation (Line(points={{11,14},
              {18,14},{18,4},{24,4}}, color={0,0,255}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{
                -120,-100},{100,100}})), Diagram(coordinateSystem(
              preserveAspectRatio=false, extent={{-100,-100},{100,100}})));
    end SundsbarmWaterway;
  end Tutorial6;

  package Tutorial7
    model PlantConnectAndDisconnectToGrid
      extends HydroPower.Examples.PlantConnectAndDisconnectToGrid;
      annotation (experiment(
          StopTime=600,
          __Dymola_NumberOfIntervals=5000,
          Tolerance=1e-05,
          __Dymola_Algorithm="Radau"));
    end PlantConnectAndDisconnectToGrid;

    model Sundsbarm
      "Hydro plant model connecting to grid at t=150s and disconnecting at t=350s"
      extends Modelon.Icons.Experiment;

      HydroPower.MechanicalSystems.BasicTurbine turbine(
        np=12,
        H_nom=480,
        tableOnFile=true,
        LdraftTube=10,
        DavDraftTube=2,
        LscrollCase=5,
        DavScrollCasing=2,
        PUInFlowTables=true,
        QTableName="Qtab",
        Q_nom=24,
        H_start=564 + 48,
        H_start_draftTube=115,
        Ty=0.4,
        yvLim1=[-0.1, 0.1],
        yvLim2=[-0.2, 0.2],
        TurbineDataFile=Modelica.Utilities.Files.loadResource(HydroPower.TABLE_DIR
             + "TurbineDataFile.mat"),
        P_nom=103000000)
                        annotation (Placement(transformation(extent={{12,-49},{
                32,-29}},
                     rotation=0)));

      HydroPower.ElectricalSystems.PowerGrid powerGrid(
        startTime=1e6,
        unitsJ={122000,5.5e6,8000},
        NoLoadUnits={200,400,1000},
        distNoGen={-2,0,0},
        distTgen={150,1e6,1e6}) annotation (Placement(transformation(extent={{-89,
                40},{-69,60}}, rotation=0)));

      Modelica.Blocks.Sources.Ramp pwr_ref(
        duration=10,
        height=0,
        offset=45e6,
        startTime=1e6) annotation (Placement(transformation(extent={{-37,64},{-25,
                76}}, rotation=0)));
      HydroPower.ElectricalSystems.GeneratorAndMCB generator(
        np={12},
        Kdmp={0.02},
        f_start=0,
        J={212500.0},
        timeMCB_close={150},
        timeMCB_open={200},
        P_nom={103000000})
                          annotation (Placement(transformation(extent={{-59,40},
                {-39,60}},
                       rotation=0)));
      HydroPower.ControllersAndSensors.TurbineGovernorAnalog turbineGovernor(
        ep=1,
        DeadBand=0.001,
        Ki_load=0.2,
        Kd_load=0.1,
        Kd_noLoad=0.1,
        Ki_noLoad=0.1,
        K_noLoad=0.8,
        K_load=0.2,
        tRamp=40,
        P_generator_nom=generator.P_nom[1],
        enableRamp=false) annotation (Placement(transformation(extent={{-24,40},
                {-4,60}}, rotation=0)));
      HydroPower.Visualizers.RealValue turbinePower(precision=2, input_Value=
            turbine.summary.P_turbine*1e-6)
        annotation (Placement(transformation(extent={{64,10},{78,24}})));
      HydroPower.Visualizers.BooleanIndicator MCB(input_Value=turbineGovernor.summary.isMCB)
        annotation (Placement(transformation(extent={{64,73},{77,87}})));
      HydroPower.Visualizers.RealValue gridbalanceNum(precision=2, input_Value=
            generator.summary.P_grid_tot*1e-6)
        annotation (Placement(transformation(extent={{64,38},{78,52}})));
      inner HydroPower.System_HPL system_HPL(
        steadyState=true,
        pipeRoughness=0.01,
        T_start=283.15,
        constantTemperature=true)
        annotation (Placement(transformation(extent={{-90,-80},{-70,-60}})));
      HydroPower.Visualizers.RealValue gridbalanceNum1(precision=2, input_Value
          =generator.summary.f[1])
        annotation (Placement(transformation(extent={{64,53},{78,67}})));
      HydroPower.Visualizers.RealValue gridbalanceNum2(precision=2, input_Value
          =generator.summary.P_generator[1]*1e-6)
        annotation (Placement(transformation(extent={{64,25},{78,39}})));
      HydroPower.SinksAndSources.Fixed_HT warterSource(
        paraOption=false,
        H_const=reservoir.H_start[reservoir.n],
        Hmax=reservoir.Hmax[reservoir.n],
        depth=reservoir.depth[1])
        annotation (Placement(transformation(extent={{-75,-18},{-95,2}})));
      HydroPower.HydroSystems.Pipe conduit(
        endD={5.8,5.8},
        L=6600,
        ZL=564,
        ZR=541.5,
        horizontalIcon=true)
        annotation (Placement(transformation(extent={{-69,-49},{-46,-29}})));
      HydroPower.HydroSystems.SurgeTank surgeTank(
        D=3.6,
        deltZ=150,
        H2L=0.87,
        Vol=100)
        annotation (Placement(transformation(extent={{-43,-49},{-23,-29}})));
      HydroPower.HydroSystems.Pipe tailRace(
        endD={5.8,5.8},
        L=600,
        ZL=110.5,
        ZR=110,
        horizontalIcon=true)
        annotation (Placement(transformation(extent={{46,-57},{66,-37}})));
      HydroPower.HydroSystems.Reservoir river(
        Hmax=ones(river.n)*(110 + 5 + 3),
        depth=ones(river.n)*(5 + 3),
        H_start=ones(river.n)*(110 + 5))
        annotation (Placement(transformation(extent={{75,-51},{95,-31}})));
      HydroPower.SinksAndSources.Fixed_HT waterSink(
        paraOption=false,
        H_const=river.H_start[1],
        Hmax=river.Hmax[1],
        depth=river.depth[river.n])
        annotation (Placement(transformation(extent={{73,-20},{93,0}})));
      HydroPower.HydroSystems.Reservoir reservoir(
        Hmax=ones(reservoir.n)*(564 + 48 + 5),
        depth=ones(reservoir.n)*(48 + 5),
        H_start=ones(reservoir.n)*(564 + 48),
        steadyState=true)
        annotation (Placement(transformation(extent={{-95,-43},{-75,-23}})));
      HydroPower.HydroSystems.Pipe pressureShaft(
        endD={3,3},
        L=724,
        ZL=541.5,
        ZR=112.5,
        horizontalIcon=false)
        annotation (Placement(transformation(extent={{-17,-49},{6,-29}})));
    equation

      connect(powerGrid.f_grid, generator.f_grid) annotation (Line(
          points={{-68,43},{-60,43}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(powerGrid.P_grid_balance, generator.P_grid_balance) annotation (Line(
          points={{-68,57},{-60,57}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(pwr_ref.y, turbineGovernor.P_reference) annotation (Line(
          points={{-24.4,70},{-20,70},{-20,61}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(generator.f_out[1], turbineGovernor.f) annotation (Line(
          points={{-38,43},{-25,43}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(generator.onMCB, powerGrid.MCB) annotation (Line(
          points={{-49,61},{-49,81},{-79,81},{-79,61}},
          color={255,0,255},
          smooth=Smooth.None));
      connect(generator.onMCB[1], turbineGovernor.isMCB) annotation (Line(
          points={{-49,61},{-49,81},{-14,81},{-14,61}},
          color={255,0,255},
          smooth=Smooth.None));
      connect(generator.P_out[1], turbineGovernor.P_generator) annotation (Line(
          points={{-38,57},{-25,57}},
          color={0,0,127},
          smooth=Smooth.None));

      connect(generator.f_out, powerGrid.f) annotation (Line(
          points={{-38,43},{-33,43},{-33,11},{-94,11},{-94,43},{-90,43}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(powerGrid.J_grid, generator.J_grid)
        annotation (Line(points={{-68,50},{-60,50}},          color={0,0,127}));
      connect(turbineGovernor.y, turbine.yGV)
        annotation (Line(points={{-3,50},{28,50},{28,-28}},
                                                        color={0,0,127}));
      connect(generator.f_out[1], turbine.f_generator) annotation (Line(points={{-38,43},
              {-33,43},{-33,11},{16,11},{16,-28}},       color={0,0,127}));
      connect(turbine.TurbineData[1], generator.P_turbine[1]) annotation (Line(
            points={{22,-28.6667},{22,26},{-57,26},{-57,39}},            color={0,0,
              127}));
      connect(reservoir.a1_open, warterSource.b) annotation (Line(points={{-96,
              -27},{-98,-27},{-98,-8},{-96,-8}}, color={0,0,255}));
      connect(reservoir.a2_pipe,conduit. a)
        annotation (Line(points={{-74,-39},{-70.15,-39}},
                                                     color={0,0,255}));
      connect(conduit.b,surgeTank. a)
        annotation (Line(points={{-44.85,-39},{-44,-39}},
                                                     color={0,0,255}));
      connect(surgeTank.b, pressureShaft.a)
        annotation (Line(points={{-22,-39},{-18.15,-39}}, color={0,0,255}));
      connect(pressureShaft.b, turbine.a)
        annotation (Line(points={{7.15,-39},{11,-39}}, color={0,0,255}));
      connect(turbine.b, tailRace.a) annotation (Line(points={{33,-39},{40,-39},
              {40,-47},{45,-47}}, color={0,0,255}));
      connect(tailRace.b, river.a1_pipe)
        annotation (Line(points={{67,-47},{74,-47}}, color={0,0,255}));
      connect(river.a2_open, waterSink.b) annotation (Line(points={{96,-35},{99,
              -35},{99,-10},{94,-10}}, color={0,0,255}));
      annotation (
        Diagram(coordinateSystem(
            preserveAspectRatio=false,
            extent={{-100,-100},{100,100}},
            grid={1,1}), graphics={
            Rectangle(
              extent={{50,91},{90,7}},
              lineColor={215,215,215},
              fillColor={215,215,215},
              fillPattern=FillPattern.Solid,
              radius=2),           Text(
              extent={{54,13},{90,9}},
              lineColor={0,0,0},
              lineThickness=0.5,
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              textString="turbine power [MW]"),Text(
              extent={{57,73},{85,65}},
              lineColor={0,0,0},
              lineThickness=0.5,
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              textString="Main Circuit Breaker"),
                                Text(
              extent={{39,41},{103,37}},
              lineColor={0,0,0},
              lineThickness=0.5,
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              textString="grid power balance [MW]"),
                                Text(
              extent={{39,56},{103,52}},
              lineColor={0,0,0},
              lineThickness=0.5,
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              textString="generator frequency [Hz]"),
                                Text(
              extent={{38,27},{102,23}},
              lineColor={0,0,0},
              lineThickness=0.5,
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              textString="generator power [MW]")}),
        experiment(
          StopTime=600,
          __Dymola_NumberOfIntervals=5000,
          Tolerance=1e-05,
          __Dymola_Algorithm="Radau"),
        Documentation(info="<html>
<p>This example illustrates a hydro power plant acting under no load and when connected to the power grid. </p>
<h4>Model experiment description</h4>
<p>When there is no load present the governor will only have the frequency error as input signal which will have the effect that the frequency of the hydro plant generator is controlled to equal the nominal frequency. This behaviour can be seen during the first 150s of simulation. </p>
<p>When 150s has passed and the frequency of the generator is synchronized to the grid frequency, the MCB is closed, the power reference is set to 45MW and new PID parameters are applied. </p>
<p>At time=350 load rejection takes place and the MCB opens once again. </p>
<h4>Simulation setup</h4>
<p>Simulate for 600s using solver Radau with a tolerance set to 1e-6.</p>
<h4>Output</h4>
<p>The most interesting variables are:</p>
<ul>
<li>generator frequency - generator.summary.f[1]</li>
<li>generated power - generator.summary.P_generator[1]</li>
</ul>
</html>",     revisions="<html>
<hr><p><font color=\"#E72614\"><b>Copyright &copy; 2004-2020, MODELON AB</b></font> <font color=\"#AFAFAF\"><br /><br /> The use of this software component is regulated by the licensing conditions for Modelon Libraries. <br /> This copyright notice must, unaltered, accompany all components that are derived from, copied from, <br /> or by other means have their origin from any Modelon Library. </font></p>
</html>"),
        Icon(coordinateSystem(
            preserveAspectRatio=false,
            extent={{-100,-100},{100,100}},
            grid={1,1})),
        __Dymola_experimentSetupOutput);
    end Sundsbarm;
  end Tutorial7;
  annotation (uses(Modelica(version="3.2.3"), HydroPower(version="2.11"),
      Modelon(version="3.5")));
end FM3217_2020;

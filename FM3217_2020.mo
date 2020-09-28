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
  annotation (uses(Modelica(version="3.2.3")));
end FM3217_2020;

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
            coordinateSystem(preserveAspectRatio=false)));
    end Motor;

    model Motordrive
      Motor motor
        annotation (Placement(transformation(extent={{0,24},{20,44}})));
      Modelica.Blocks.Math.Feedback positionerror
        annotation (Placement(transformation(extent={{-66,24},{-46,44}})));
      Modelica.Blocks.Sources.Step step
        annotation (Placement(transformation(extent={{-94,24},{-74,44}})));
      Modelica.Blocks.Continuous.PID controller
        annotation (Placement(transformation(extent={{-32,24},{-12,44}})));
      Modelica.Mechanics.Rotational.Components.IdealGear gearbox(ratio=100)
        annotation (Placement(transformation(extent={{30,24},{50,44}})));
      Modelica.Mechanics.Rotational.Components.Inertia load(J=5)
        annotation (Placement(transformation(extent={{58,24},{78,44}})));
      Modelica.Mechanics.Rotational.Sensors.AngleSensor angleSensor annotation
        (Placement(transformation(
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
  annotation (uses(Modelica(version="3.2.3")));
end FM3217_2020;

within ;
model SimplePendulum  "Model of a simple pendulum"
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

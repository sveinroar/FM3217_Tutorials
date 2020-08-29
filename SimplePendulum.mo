within ;
model SimplePendulum
  constant Real g = 9.81;
  parameter Real L = 1;
  Real Theta;
  Real ThetaDot;
equation
  ThetaDot = der(Theta);
  der(ThetaDot) = -g/L*sin(Theta);
end SimplePendulum;

package propeller_env
   
class propeller
    //Parameter
    parameter Real c_w = 0.5 "Drag coefficient";
    parameter Modelica.Units.SI.Diameter d = 0.3 "Diameter";
    parameter Real n_o_b = 2 "Number of blades";
    parameter Modelica.Units.SI.Area A_p = 0.13 "Area of the propeller";
    annotation(
      Icon(graphics = {Line(origin = {6.05, -26.45}, points = {{-15, 24}, {-15, -24}, {15, -24}, {15, 22}, {-15, 22}, {-1, 22}}, thickness = 0.5), Ellipse(origin = {-42, -4}, extent = {{36, -6}, {-36, 6}}), Ellipse(origin = {-42, -4}, extent = {{36, -6}, {-36, 6}}), Ellipse(origin = {-42, -4}, extent = {{36, -6}, {-36, 6}}), Ellipse(origin = {56, -6}, extent = {{36, -6}, {-36, 6}})}));
  end propeller;

  model propeller_mod
  propeller_env.propeller propeller annotation(
      Placement(visible = true, transformation(origin = {0, 34}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  
  //Constans
  import Modelica.Constants.g_n; //Gravitational acceleration
  //Parameter
  parameter Real rho(unit = "kg/m^3") = 1.225 "Air density";
  parameter Modelica.Units.SI.Mass m_g = 0.5 "Mass of the System";
  parameter Modelica.Units.si.Velocity v_e_max = 8 "maximum climbing velocity of drone";
  //Variable
  Modelica.Units.SI.Velocity v_s "Velocity of the Sytem";
  Modelica.Units.SI.Velocity v_e "Velocity of the incoming Wind (Environment/Propeller)";
  Modelica.Units.SI.Velocity v_r "Velocity of the outgoing Wind (Propeller)";
  Modelica.Units.SI.Force F_s "Force of the System";
  Modelica.Units.SI.Force F_d "Drag Force";
  Modelica.Units.SI.Acceleration a_s "Acceleration of the System";
  Real m_dot (unit = "kg/sec") "Mass flow";

  equation
  F_s = m_g*a_s; //Newton's second law
  F_d = 0.5*rho*v_s^2*propeller.c_w*propeller.d^2*propeller.n_o_b; //Drag force
  m_dot = propeller.A_p*rho*v_r; //Mass flow


  end propeller_mod;
end propeller_env;

within wsm_simulations;

model ros_inverted_pendelum_reward_function
  Modelica.Blocks.Interfaces.RealInput position annotation(Placement(visible = true, transformation(origin = {-39.464, 57.296}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-90, 55}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput velocity annotation(Placement(visible = true, transformation(origin = {-39.464, 57.296}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-90, 18}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput angularVelocity annotation(Placement(visible = true, transformation(origin = {-40, 27.763}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-90, -18}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput angle annotation(Placement(visible = true, transformation(origin = {-40, -5}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-90, -55}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput reward annotation(Placement(visible = true, transformation(origin = {170.524, -12.57}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, -0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
protected
  Real absPosition;
  Real absVelocity;
  Real absAngle;
equation
  absPosition = if position < 0 then -position else position;
  absVelocity = if velocity < 0 then -velocity else velocity;
  absAngle = if angle < 0 then -angle else angle;
  reward = 1.0 / (1.00 + Modelica.Math.exp(2.0 * absAngle)) - 0 * absVelocity ^ 2;
  annotation(Diagram(coordinateSystem(extent = {{-150, -90}, {150, 90}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})), Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10}), graphics = {Rectangle(visible = true, lineColor = {0, 114, 195}, fillColor = {160, 160, 164}, fillPattern = FillPattern.Solid, extent = {{-100, -100}, {100, 100}}, radius = 25), Text(visible = true, textColor = {64, 64, 64}, extent = {{-150, 110}, {150, 150}}, textString = "%name"), Text(visible = true, origin = {1.175, 11.633}, textColor = {255, 255, 255}, extent = {{-88.825, -68.367}, {88.825, 68.367}}, textString = "r")}));
end ros_inverted_pendelum_reward_function;

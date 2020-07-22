within wsm_simulations;

model ros_inverted_pendelum_trainer
  ros_inverted_pendelum_controller ros_inverted_pendelum_controller1 annotation(Placement(visible = true, transformation(origin = {20, 15}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  ros_inverted_pendelum_relay ros_inverted_pendelum_relay1 annotation(Placement(visible = true, transformation(origin = {-20, 15}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  annotation(Diagram(coordinateSystem(extent = {{-55, -5}, {57.63, 36.99}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})), Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10}), graphics = {Rectangle(visible = true, lineColor = {0, 114, 195}, fillColor = {255, 255, 255}, extent = {{-100, -100}, {100, 100}}, radius = 25), Text(visible = true, textColor = {64, 64, 64}, extent = {{-150, 110}, {150, 150}}, textString = "%name")}));
end ros_inverted_pendelum_trainer;

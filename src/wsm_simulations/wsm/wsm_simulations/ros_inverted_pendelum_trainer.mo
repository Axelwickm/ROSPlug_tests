within wsm_simulations;

model ros_inverted_pendelum_trainer
  extends Modelica.Icons.References;
  ros_inverted_pendelum_controller ros_inverted_pendelum_controller1(loadModel = false, saveModel = true, internalMessagingPolicy = internalMessagingPolicy, savingInterval = 2000, training = true) annotation(Placement(visible = true, transformation(origin = {20, 15}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  ros_inverted_pendelum_relay ros_inverted_pendelum_relay1(internalMessagingPolicy = internalMessagingPolicy) annotation(Placement(visible = true, transformation(origin = {-20, 15}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  parameter ROSPlug.Publisher.InternalMessagingPolicy internalMessagingPolicy = ROSPlug.Publisher.InternalMessagingPolicy.Prohibited "If message are sent directly to subscribers within the model without passing through ROS. (publisher1.internalMessagingPolicy) (ros_inverted_pendelum_controller1.internalMessagingPolicy)";
  annotation(Diagram(coordinateSystem(extent = {{-55, -5}, {57.63, 36.99}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})), Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10}), graphics = {Rectangle(visible = true, lineColor = {0, 114, 195}, fillColor = {255, 255, 255}, extent = {{-100, -100}, {100, 100}}, radius = 25), Text(visible = true, textColor = {64, 64, 64}, extent = {{-150, 110}, {150, 150}}, textString = "%name")}));
end ros_inverted_pendelum_trainer;

<%
    arm_height = 0.5
    arm_width = 0.1
%>


<box2d>
  <world timestep="0.05">
    <body name="arm1" type="dynamic" position="0,${arm_height/2}">
      <fixture
              density="1"
              shape="polygon"
              box="${arm_width/2},${arm_height/2}"
      />
    </body>
    <body name="arm2" type="dynamic" position="0,${arm_height/2}">
      <fixture
              density="1"
              shape="polygon"
              box="${arm_width/2},${arm_height/2}"
      />
    </body>

    <joint type="" name="endPoint" bodyA="arm1" bodyB="arm2" anchor=""/>
  </world>
</box2d>
<%
    from rllab.misc.mako_utils import compute_rect_vertices
    link_len = opts['link_len']
    arm_width = 0.1
%>

<box2d>
  <world timestep="0.01" velitr="20" positr="20">
    <body name="arm1" type="dynamic" position="0,2">
      <fixture
              density="1.0"
              group="-1"
              shape="polygon"
              vertices="${compute_rect_vertices([0,0], [0, link_len], arm_width/2)}"
      />
    </body>
    <body name="arm2" type="dynamic" position="0,3" >
      <fixture
              density="1.0"
              group="-1"
              shape="polygon"
              vertices="${compute_rect_vertices([0,0], [0,link_len], arm_width/2)}"
      />
    </body>
    <body name="point" type="static" position="1 0.6">
      <fixture group="-1" shape="polygon" box="0.1 0.1"/>
    </body>
    <body name="track" type="static" position="0,1.99">
      <fixture group="-1" shape="polygon" box="0.01,0.01"/>
    </body>
    <joint type="revolute" name="arm_joint_1" bodyA="track" bodyB="arm1" anchor="0,2"/>
    <joint type="revolute" name="arm_joint_2" bodyA="arm1" bodyB="arm2" anchor="0,3"/>
    <!-- <control type="torque" joint="link_joint_1" ctrllimit="-3,3"/> -->
    <control type="torque" joint="arm_joint_2" ctrllimit="-50,50" />
    <state type="apos" body="arm1" transform="sin"/>
    <state type="apos" body="arm1" transform="cos"/>
    <state type="avel" body="arm1"/>
    <state type="apos" body="arm2" transform="sin"/>
    <state type="apos" body="arm2" transform="cos"/>
    <state type="avel" body="arm2"/>
  </world>
</box2d>

import argparse

# Need to manually delete state at this stage
# remove wind/
# remove spherical coordinates
# remove audio
def update_sdf_version(file_path, output_path=None):
    # Read the file
    with open(file_path, 'r') as file:
        content = file.read()

    updated_content = content

    # Replace the version
    updated_content = updated_content.replace("<sdf version='1.7'>", "<sdf version='1.9'>")

    # Replace the scene
    old = """<scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>1</shadows>
    </scene>"""

    new = """<scene>
      <ambient>1 1 1 1</ambient>
      <background>0.8 0.8 0.8 1</background>
      <shadows>true</shadows>
    </scene>"""

    updated_content = updated_content.replace(old, new)

    # Replace the materials
    old = """<script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>"""
    
    new = """<ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.8 0.8 0.8 1</specular>"""

    updated_content = updated_content.replace(old, new)

    old = """<script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Wood</name>
            </script>"""
    
    updated_content = updated_content.replace(old, new)
    
    old = """<script>
              <name>Gazebo/Grey</name>
              <uri>file://media/materials/scripts/gazebo.material</uri>
            </script>"""

    updated_content = updated_content.replace(old, new)

    # Ground place
    old = """<model name='ground_plane'>
      <static>1</static>
      <link name='link'>
        <collision name='collision'>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>100</mu>
                <mu2>50</mu2>
              </ode>
              <torsional>
                <ode/>
              </torsional>
            </friction>
            <contact>
              <ode/>
            </contact>
            <bounce/>
          </surface>
          <max_contacts>10</max_contacts>
        </collision>
        <visual name='visual'>
          <cast_shadows>0</cast_shadows>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.8 0.8 0.8 1</specular>
          </material>
        </visual>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
    </model>"""

    new = """<model name='ground_plane'>
      <static>true</static>
      <link name='link'>
        <collision name='collision'>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode/>
            </friction>
            <bounce/>
            <contact/>
          </surface>
        </collision>
        <visual name='visual'>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.8 0.8 0.8 1</specular>
          </material>
        </visual>
        <pose>0 0 0 0 -0 0</pose>
        <inertial>
          <pose>0 0 0 0 -0 0</pose>
          <mass>1</mass>
          <inertia>
            <ixx>1</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>1</iyy>
            <iyz>0</iyz>
            <izz>1</izz>
          </inertia>
        </inertial>
        <enable_wind>false</enable_wind>
      </link>
      <pose>0 0 0 0 -0 0</pose>
      <self_collide>false</self_collide>
    </model>"""

    updated_content = updated_content.replace(old, new)

    # Write back to the same file or a new one
    if output_path is None:
        output_path = file_path

    with open(output_path, 'w') as file:
        file.write(updated_content)

    print(f"SDF version updated in: {output_path}")

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Update to gazebo ignition')
    parser.add_argument('file', help='Path to the input .sdf file')
    parser.add_argument('-o', '--output', help='Path to output file (optional). Defaults to overwriting the input file.')

    args = parser.parse_args()
    update_sdf_version(args.file, args.output)

# -*- coding: utf-8 -*-
"""
Visualizer module for 3D rendering of visualization.

Created on Sat May  2 14:50:24 2020
@author: Vojtech Vrba, FEE CTU in Prague, vrbavoj3@fel.cvut.cz
"""

import pyglet
import ratcave as rc
import numpy as np
import json

class Object3D():
    def __init__(self, obj_model, parent, obj_name, data_json):
        Visualizer.print("Constructing mesh: %s" % obj_name)
        self.pivot_pos = np.array(data_json["position"])[[0, 2, 1]]
        self.pivot_pos[2] *= -1
        self.parent_obj = parent
        
        self.pivot = rc.EmptyEntity(name=obj_name + "_pivot")
        
        parent_pivot_pos = np.zeros((1, 3))
        if self.parent_obj is not None:
            parent_pivot_pos = self.parent_obj.get_pivot_pos_abs()
            
        self.pivot.position.xyz = tuple(self.pivot_pos - parent_pivot_pos)
        
        self.joint = Visualizer.PRIMITIVES.get_mesh("Sphere", name=obj_name + "_joint", scale=0.1)
        self.joint.position.xyz = (0, 0, 0)
        self.pivot.add_child(self.joint)
            
        self.mesh = obj_model.get_mesh(obj_name, name=obj_name)
        #Visualizer.print("Former mesh pos:", self.mesh.position.xyz, "; pivot pos:", tuple(self.pivot_pos), "; diff:", tuple(np.array(self.mesh.position.xyz) - self.pivot_pos))
        self.mesh.position.xyz = tuple(np.array(self.mesh.position.xyz) - self.pivot_pos)
        self.mesh.uniforms['diffuse'] = [.5, .0, .8]
        
        self.pivot.add_child(self.mesh)
        
        if "children" in data_json.keys() and len(data_json["children"]) > 0:
            for ch in data_json["children"]:
                o = Object3D(obj_model, self, ch, data_json["children"][ch])
                self.pivot.add_child(o.get_pivot())
                #self.children.append(o)
        else:
            self.mesh.uniforms['diffuse'] = [.4, .5, .0]
            
        Visualizer.HAND_PARTS[obj_name] = self
    
    def get_pivot_pos_abs(self):
        return self.pivot_pos
    
    def get_pivot(self):
        return self.pivot
    
    def get_mesh(self):
        return self.mesh
    
    
class Visualizer():
    PRIMITIVES = rc.WavefrontReader(rc.resources.obj_primitives)
    HAND_PARTS = dict()
    LOGGER = None
    SHARED_MEM = None
    
    def __init__(self, logger, shared_mem):
        Visualizer.LOGGER = logger
        Visualizer.SHARED_MEM = shared_mem
        
        # Create Window
        window = pyglet.window.Window(caption="Glove Visualizer (BP Vojtech Vrba, 2020)", width=1280, height=720, resizable=True)
        keys = pyglet.window.key.KeyStateHandler()
        window.push_handlers(keys)
        self.window_handle = window
        
        self.label =  pyglet.text.HTMLLabel(
            '(Empty)',
            x=window.width//2, y=window.height-10,
            anchor_x='center', anchor_y='center')
        
        
        self.table = pyglet.text.document.FormattedDocument(self.format_table_content([]))
        self.table.set_style(0, len(self.table.text), dict(font_name="Courier New", color=(0,0,0,255), wrap=False))
        self.table_layout = pyglet.text.layout.TextLayout(self.table, 240, window.height, multiline=True)
        
        # Create Meshes
        hand_model = rc.WavefrontReader("hand.obj")
        with open('hand.json') as json_file:
            Object3D(hand_model, None, "hand", json.load(json_file)["hand"])
            Visualizer.HAND_PARTS["hand"].get_pivot().position.xyz = (0, 0, -5)
            Visualizer.HAND_PARTS["hand"].get_pivot().rotation.xyz = (0, 180, 0)
        
        # Create Scene
        scene = rc.Scene(meshes=Visualizer.HAND_PARTS["hand"].get_pivot(), bgColor=(1,1,1))
        scene.camera.projection.z_far = 20
              
        # Run visualizer
        pyglet.clock.schedule(self.update)
        pyglet.clock.schedule(self.move_camera)
        
        @window.event
        def on_draw():
            with rc.default_shader:
                scene.draw()
            self.label.draw()
            self.table_layout.draw()
                
        @window.event
        def on_resize(width, height):
            scene.camera.projection.match_aspect_to_viewport()
            
        @window.event
        def on_close():
            Visualizer.print("Window closed!")
            self.stop()
    
    def format_table_content(self, sensors_angles):
        s = ["+----+-------+-------+-------+",
             "| ID | Pitch |  Yaw  | Roll  |",
             "+----+-------+-------+-------+"]
          
        for i, sa in enumerate(sensors_angles):
            s.extend(["| %02d | %+04d° | %+04d° | %+04d° |" % (i, int(sa[0]), int(sa[1]), int(sa[2])),
                      "+----+-------+-------+-------+"])
        
        return '\n'.join(s)
        
    def update(self, dt):
        values = np.array(Visualizer.SHARED_MEM)
        time = values[0] / 1000
        voltage = values[1]
        temperature = values[47]
        pressure = values[48]
        sensors_angles = np.split(values[2:47], 15)
        self.label.text = "<b>Time:</b> %03.2f s | <b>Voltage:</b> %01.2f V | <b>Temperature:</b> %02.1f °C | <b>Pressure:</b> %04.2f hPa" % (time, voltage, temperature, pressure)
        
        Visualizer.HAND_PARTS["thumb1"].get_pivot().rotation.xyz = tuple(sensors_angles[0])
        Visualizer.HAND_PARTS["thumb2"].get_pivot().rotation.xyz = tuple(sensors_angles[0])
        Visualizer.HAND_PARTS["thumb3"].get_pivot().rotation.xyz = tuple(sensors_angles[1])
        Visualizer.HAND_PARTS["index2"].get_pivot().rotation.xyz = tuple(sensors_angles[2])
        Visualizer.HAND_PARTS["index3"].get_pivot().rotation.xyz = tuple(sensors_angles[3])
        Visualizer.HAND_PARTS["index1"].get_pivot().rotation.xyz = tuple(sensors_angles[4])
        Visualizer.HAND_PARTS["middle1"].get_pivot().rotation.xyz = tuple(sensors_angles[5])
        Visualizer.HAND_PARTS["middle2"].get_pivot().rotation.xyz = tuple(sensors_angles[6])
        Visualizer.HAND_PARTS["middle3"].get_pivot().rotation.xyz = tuple(sensors_angles[7])
        Visualizer.HAND_PARTS["ring2"].get_pivot().rotation.xyz = tuple(sensors_angles[8])
        Visualizer.HAND_PARTS["ring3"].get_pivot().rotation.xyz = tuple(sensors_angles[9])
        Visualizer.HAND_PARTS["ring1"].get_pivot().rotation.xyz = tuple(sensors_angles[10])
        Visualizer.HAND_PARTS["little1"].get_pivot().rotation.xyz = tuple(sensors_angles[11])
        Visualizer.HAND_PARTS["little2"].get_pivot().rotation.xyz = tuple(sensors_angles[12])
        Visualizer.HAND_PARTS["little3"].get_pivot().rotation.xyz = tuple(sensors_angles[13])
        Visualizer.HAND_PARTS["hand"].get_pivot().rotation.xyz = tuple(sensors_angles[14])
        
        self.table.text = self.format_table_content(sensors_angles)
    
        
    def move_camera(self, dt):
      camera_speed = 3
      #if keys[pyglet.window.key.LEFT]:
          #hand.get_pivot().rotation.x += 1
          #scene.camera.x -= camera_speed * dt
      #if keys[pyglet.window.key.RIGHT]:
          #hand.get_pivot().rotation.y += 1
          #scene.camera.x += camera_speed * dt
        
    def run(self):
        pyglet.app.run()
    
    def stop(self):
        #self.window_handle.close()
        pyglet.app.exit()
        
    def get_window_handle(self):
        return self.window_handle
    
    @staticmethod
    def print(s, debug=False):
        if Visualizer.LOGGER is None or debug:
            print(s)
        else:
            Visualizer.LOGGER.send(s)
        

if __name__ == "__main__":
    Visualizer(None, [0.0] * (15*3+4)).run()
    
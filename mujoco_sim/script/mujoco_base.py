#!/usr/bin/env python3
import mujoco as mj
from mujoco.glfw import glfw
import rclpy
from std_msgs.msg import Bool 
import time

class MuJoCoBase():
    def __init__(self, xml_path, node):
        self.node = node
        # For callback functions
        self.button_left = False
        self.button_middle = False
        self.button_right = False
        self.lastx = 0
        self.lasty = 0
        self.pause_flag = True
        self.controller_ready = False
        self.controller_ready_time = None
        self.ready_hold_duration = 2.0
        self.pending_unpause_request = False
        self.startup_auto_unpause_pending = True
        self.space_pressed = False
        self.keyboard_enable_time = time.time() + 2.0
        self.pubSimState = self.node.create_publisher(Bool, '/pauseFlag', 10)
        self.node.create_subscription(Bool, '/pauseFlag', self.pause_callback, 10)
        self.node.create_subscription(Bool, '/controllerReady', self.controller_ready_callback, 10)
        # MuJoCo data structures
        self.model = mj.MjModel.from_xml_path(xml_path)  # MuJoCo model
        self.data = mj.MjData(self.model)                # MuJoCo data
        self.cam = mj.MjvCamera()                        # Abstract camera
        self.opt = mj.MjvOption()                        # visualization options

        # Init GLFW, create window, make OpenGL context current, request v-sync
        glfw.init()
        self.window = glfw.create_window(1200, 900, "Demo", None, None)
        glfw.make_context_current(self.window)
        glfw.swap_interval(1)

        # initialize visualization data structures
        mj.mjv_defaultCamera(self.cam)
        mj.mjv_defaultOption(self.opt)
        self.scene = mj.MjvScene(self.model, maxgeom=10000)
        self.context = mj.MjrContext(
            self.model, mj.mjtFontScale.mjFONTSCALE_150.value)

        # install GLFW mouse and keyboard callbacks
        glfw.set_key_callback(self.window, self.keyboard)
        glfw.set_cursor_pos_callback(self.window, self.mouse_move)
        glfw.set_mouse_button_callback(self.window, self.mouse_button)
        glfw.set_scroll_callback(self.window, self.scroll)

    def pause_callback(self, msg):
        requested_pause = bool(msg.data)
        if not requested_pause and not self.can_run_simulation():
            self.pause_flag = True
            self.pending_unpause_request = True
            return
        self.pending_unpause_request = False
        self.pause_flag = requested_pause

    def controller_ready_callback(self, msg):
        was_ready = self.controller_ready
        self.controller_ready = bool(msg.data)
        if self.controller_ready and not was_ready:
            self.controller_ready_time = time.time()
            self.node.get_logger().info('Controller initialization finished.')

    def can_run_simulation(self):
        if not self.controller_ready or self.controller_ready_time is None:
            return False
        return time.time() >= self.controller_ready_time + self.ready_hold_duration

    def release_pending_unpause_if_ready(self):
        if self.pending_unpause_request and self.can_run_simulation():
            self.pending_unpause_request = False
            self.startup_auto_unpause_pending = False
            self.pause_flag = False
            self.node.get_logger().info('Applying queued unpause request.')

    def release_startup_pause_if_ready(self):
        if self.startup_auto_unpause_pending and not self.pending_unpause_request and self.can_run_simulation():
            self.startup_auto_unpause_pending = False
            self.pause_flag = False
            self.node.get_logger().info('Automatically releasing startup pause.')
            simState = Bool()
            simState.data = self.pause_flag
            self.pubSimState.publish(simState)

    def keyboard(self, window, key, scancode, act, mods):
        if act == glfw.PRESS and key == glfw.KEY_BACKSPACE:
            mj.mj_resetData(self.model, self.data)
            mj.mj_forward(self.model, self.data)
        elif key == glfw.KEY_SPACE:
            if time.time() < self.keyboard_enable_time:
                self.space_pressed = False
                return
            if act == glfw.PRESS:
                self.space_pressed = True
                return
            if act != glfw.RELEASE or not self.space_pressed:
                return
            self.space_pressed = False
            self.pause_flag = not self.pause_flag
            if not self.pause_flag and not self.can_run_simulation():
                self.pause_flag = True
                self.node.get_logger().info('Unpause requested before controller startup settled; staying paused.')
            else:
                self.startup_auto_unpause_pending = False
            mj.mj_forward(self.model, self.data)
            simState = Bool()
            simState.data = self.pause_flag
            self.pubSimState.publish(simState)
            

    def mouse_button(self, window, button, act, mods):
        # update button state
        self.button_left = (glfw.get_mouse_button(
            window, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS)
        self.button_middle = (glfw.get_mouse_button(
            window, glfw.MOUSE_BUTTON_MIDDLE) == glfw.PRESS)
        self.button_right = (glfw.get_mouse_button(
            window, glfw.MOUSE_BUTTON_RIGHT) == glfw.PRESS)

        # update mouse position
        glfw.get_cursor_pos(window)

    def mouse_move(self, window, xpos, ypos):
        # compute mouse displacement, save
        dx = xpos - self.lastx
        dy = ypos - self.lasty
        self.lastx = xpos
        self.lasty = ypos

        # no buttons down: nothing to do
        if (not self.button_left) and (not self.button_middle) and (not self.button_right):
            return

        # get current window size
        width, height = glfw.get_window_size(window)

        # get shift key state
        PRESS_LEFT_SHIFT = glfw.get_key(
            window, glfw.KEY_LEFT_SHIFT) == glfw.PRESS
        PRESS_RIGHT_SHIFT = glfw.get_key(
            window, glfw.KEY_RIGHT_SHIFT) == glfw.PRESS
        mod_shift = (PRESS_LEFT_SHIFT or PRESS_RIGHT_SHIFT)

        # determine action based on mouse button
        if self.button_right:
            if mod_shift:
                action = mj.mjtMouse.mjMOUSE_MOVE_H
            else:
                action = mj.mjtMouse.mjMOUSE_MOVE_V
        elif self.button_left:
            if mod_shift:
                action = mj.mjtMouse.mjMOUSE_ROTATE_H
            else:
                action = mj.mjtMouse.mjMOUSE_ROTATE_V
        else:
            action = mj.mjtMouse.mjMOUSE_ZOOM

        mj.mjv_moveCamera(self.model, action, dx/height,
                          dy/height, self.scene, self.cam)

    def scroll(self, window, xoffset, yoffset):
        action = mj.mjtMouse.mjMOUSE_ZOOM
        mj.mjv_moveCamera(self.model, action, 0.0, -0.05 *
                          yoffset, self.scene, self.cam)

    def simulate(self):
        while not glfw.window_should_close(self.window):
            simstart = self.data.time

            while (self.data.time - simstart < 1.0/60.0):
                mj.mj_step(self.model, self.data)

            # get framebuffer viewport
            viewport_width, viewport_height = glfw.get_framebuffer_size(
                self.window)
            viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)

            # Update scene and render
            mj.mjv_updateScene(self.model, self.data, self.opt, None, self.cam,
                               mj.mjtCatBit.mjCAT_ALL.value, self.scene)
            mj.mjr_render(viewport, self.scene, self.context)

            # swap OpenGL buffers (blocking call due to v-sync)
            glfw.swap_buffers(self.window)

            # process pending GUI events, call GLFW callbacks
            glfw.poll_events()

        glfw.terminate()

    def reset(self, *args, **kwargs):
        raise NotImplementedError

    def controller(self, *args, **kwargs):
        raise NotImplementedError

# This script implements levitating objects using a Vive controller.
# Instance this scene as a child to your ARVRController node.
#
# The trigger grabs an object and removes any gravitational influences
# on it. Hold the grip button to grab all objects in a cone-shaped area
# extruding from your hand.
#
# The amount of force applied to the objects (both angular and linear)
# depends on how far the trigger is pulled. Under 0.3 the object is not
# moved, only levitated, allowing you to reposition your controller.

extends Area


# We need this inner class for cone shaped detection of objects.
class ConeShape:
    extends ConvexPolygonShape

    export var radius = 1 #setget set_radius
    func set_radius(r):
        radius = r
        _init()

    export var length = 1 #setget set_length
    func set_length(l):
        length = l
        _init()

    export var edge_count = 8 #setget set_edge_count
    func set_edge_count(c):
        edge_count = c
        _init()


    # Makes a cone facing the -z direction (forwards).
    # Points are the amount of corners defining the round end of the cone.
    static func make_cone(length, radius, edge_count) -> PoolVector3Array:
        var ps : PoolVector3Array = PoolVector3Array([Vector3(0,0,0)])
        var ang : float = 2 * PI / edge_count
        var rot_axis : Vector3 = Vector3(0,0,1)
        var pre_rot : Vector3 = Vector3(radius, 0, -length)
        for p in range(edge_count + 1):
            ps.push_back(pre_rot.rotated(rot_axis, ang * p))
        return ps


    func _init(length : float = 1, radius : float = 1, edge_count : int = 8) -> void:
        self.points = make_cone(length, radius, edge_count)


export var rumbling_enabled : bool = true


export var max_range : float = 15 setget set_max_range
func set_max_range(mr : float):
    max_range = mr
    # Won't exist until after _ready
    if self.force_ray:
        self.force_ray.cast_to.z = -mr
    # Won't exist until after _ready
    if self.force_cone and self.force_cone.shape:
        self.force_cone.shape = generate_shape()


export var cone_angle : float = deg2rad(15) setget set_cone_angle
func set_cone_angle(cang : float):
    cone_angle = cang
    # Won't exist until after _ready
    if self.force_cone and self.force_cone.shape:
        self.force_cone.shape = generate_shape()

export var strength : float = 15


onready var controller : ARVRController = self.get_parent()

onready var force_cone : CollisionShape = $CollisionShape

onready var force_ray : RayCast = $RayCast


var lift_multiple : bool = false

var caught_objects : Array = []

var objects_in_cone : Array = []

var use_force : bool = false setget set_use_force
func set_use_force(is_use):
    use_force = is_use
    if is_use:
        self.force_ray.enabled = !self.lift_multiple
    else:
        self.force_ray.enabled = false
        drop_all()

# Direction we're facing when first picking up an object.
# Used to disable multi-lift after moving the controller.
# Don't want to pick up additional objects when already moving some around.
var initial_orient : Vector3


func generate_shape() -> ConeShape:
    var radius : float = sin(cone_angle) * max_range / 2
    return ConeShape.new(self.max_range, radius, 8)


func drop_all():
    for body in self.caught_objects:
        # Let's hope nobody had these set to anything else :)
        body.gravity_scale = 1
        body.linear_damp = -1
        body.angular_damp = -1
    self.caught_objects = []


func lift_object(body : RigidBody) -> void:
    body.linear_damp = 0.25
    body.angular_damp = 0.7
    body.gravity_scale = 0
    self.initial_orient = -self.global_transform.basis.z
    self.caught_objects.push_back(body)
    self.force_ray.enabled = false


func manipulate_objects(objects : Array, trigger_pull : float, delta : float) -> float:
    var self_pos_diff : Vector3 = self.global_transform.origin - last_hand_transform.origin
    var applied_force : float = 0

    var power : float = pow(trigger_pull,2) * self.strength

    for body in objects:
        applied_force += body.weight

        # Move object if trigger is pulled at least a bit.
        # Below this threshold the object will only levitate,
        # allowing you to reposition your controller freely.
        if trigger_pull > 0.3:
            # Rotate object
            var last_bs = last_hand_transform.basis
            var bs = self.global_transform.basis
            var k = body.mass * power * 0.005
            var quat_diff = (Quat(bs) * Quat(last_bs).inverse()).normalized()
            var t_impulse = quat_diff.get_euler() * k
            body.apply_torque_impulse(t_impulse)

            # Move object
            var motion : Vector3 =  power * self_pos_diff
            var impulse : Vector3 = body.mass * motion
            body.apply_impulse(Vector3(), impulse)

            applied_force += body.weight + body.mass * (t_impulse.length() + impulse.length())

    return applied_force


func _on_body_exited(body : PhysicsBody) -> void:
    self.objects_in_cone.erase(body)


func _on_body_entered(body : PhysicsBody) -> void:
    if body is RigidBody:
        self.objects_in_cone.push_back(body)


func _on_button_pressed(button : int) -> void:
    match button:
        2: # Grip button on the Vive controller
            self.lift_multiple = true


func _on_button_release(button : int) -> void:
    match button:
        2:
            self.lift_multiple = false


func _ready() -> void:
    self.force_ray.enabled = false
    self.max_range = self.max_range
    self.cone_angle = self.cone_angle
    self.force_cone.shape = generate_shape()
    self.connect("body_entered", self, "_on_body_entered")
    self.connect("body_exited", self, "_on_body_exited")
    controller.connect("button_pressed", self, "_on_button_pressed")
    controller.connect("button_release", self, "_on_button_release")


# Keep track of our movement
onready var last_hand_transform : Transform = self.global_transform
func _physics_process(delta : float) -> void:
    if not controller.get_is_active():
        return

    var trigger_pull : float = controller.get_joystick_axis(2)

    # We don't want to overuse the setter, so check if we need to change it first.
    var trigger_pulled : bool = trigger_pull > 0.05
    if use_force != trigger_pulled:
        self.use_force = trigger_pulled

    # For tactile feedback.
    var applied_force : float = 0

    if self.use_force:
        # This will only do anything when the force ray is enabled.
        if self.force_ray.enabled and self.force_ray.is_colliding():
            var new_object : PhysicsBody = self.force_ray.get_collider()
            if new_object and new_object is RigidBody:
                lift_object(new_object)

        # If we've moved our controller away while multi-lifting,
        # stop catching new objects because that's annyoing.
        var self_fw : Vector3 = -self.global_transform.basis.z
        var turned_away = self.initial_orient.angle_to(self_fw) > self.cone_angle / 2
        if self.lift_multiple and not turned_away:
            for body in self.objects_in_cone:
                if not self.caught_objects.has(body):
                    lift_object(body)

        # Manipulate all objects you've picked up.
        # If lifting multiple is disabled it should only contain one object.
        if not self.caught_objects.empty():
            applied_force = manipulate_objects(self.caught_objects, trigger_pull, delta)

    # Some tactile feedback
    if self.rumbling_enabled:
        controller.rumble = trigger_pull * 0.005 * applied_force \
            if self.use_force \
            else 0.0

    # Update last global tranform
    self.last_hand_transform = self.global_transform
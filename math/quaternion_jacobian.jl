using Symbolics
using Latexify

function quat_mult(q_1, q_2)
    # Quaternion multiplication: q_1 * q_2
    return [
        q_1[1]*q_2[1] - q_1[2]*q_2[2] - q_1[3]*q_2[3] - q_1[4]*q_2[4],
        q_1[1]*q_2[2] + q_1[2]*q_2[1] + q_1[3]*q_2[4] - q_1[4]*q_2[3],
        q_1[1]*q_2[3] - q_1[2]*q_2[4] + q_1[3]*q_2[1] + q_1[4]*q_2[2],
        q_1[1]*q_2[4] + q_1[2]*q_2[3] - q_1[3]*q_2[2] + q_1[4]*q_2[1]
    ]
end

# Define symbolic variables
@variables q_0 q_1 q_2 q_3 real=true  # quaternion components
@variables x y z real=true        # components of the 3D vector

# Define the quaternion
q = [q_0, q_1, q_2, q_3]

# Define the 3D vector
v = [x, y, z]

# Down vector (commented out)
# v[1] = 0
# v[2] = 0
# v[3] = 1

# Quaternion multiplication (q*v*q')
q_conj = [q_0, -q_1, -q_2, -q_3]  # Conjugate of the quaternion

# Quaternion-vector multiplication q*v
v_quat = [0; v]  # Extend vector to quaternion form
qv = quat_mult(q, v_quat)  # Quaternion multiplication q*v

# Quaternion-vector multiplication (q*v)*q'
v_rot_quat = quat_mult(qv, q_conj)

# Extract rotated vector
v_rot = v_rot_quat[2:4]

# Compute the Jacobian of the rotated vector with respect to the quaternion
J = Symbolics.jacobian(v_rot, q)

# Simplify the Jacobian by factoring common terms
J_simplified = Symbolics.simplify.(J)

# Display the Jacobian
println("Jacobian of the rotated vector with respect to the quaternion:")
println(J_simplified)

# Display the Jacobian ... nicer.
println("Jacobian of the rotated vector with respect to the quaternion:")
latexify(J_simplified) |> println

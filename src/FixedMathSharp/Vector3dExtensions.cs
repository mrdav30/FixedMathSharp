namespace FixedMathSharp
{
    public static class Vector3dExtensions
    {
        /// <summary>
        /// Clamps each component of the vector to the range [-1, 1] in place and returns the modified vector.
        /// </summary>
        /// <param name="v">The vector to clamp.</param>
        /// <returns>The clamped vector with each component between -1 and 1.</returns>
        public static Vector3d ClampOneInPlace(this Vector3d v)
        {
            v.x = v.x.ClampOne();
            v.y = v.y.ClampOne();
            v.z = v.z.ClampOne();
            return v;
        }

        /// <summary>
        /// Converts each component of the vector from radians to degrees.
        /// </summary>
        /// <param name="radians">The vector with components in radians.</param>
        /// <returns>A new vector with components converted to degrees.</returns>
        public static Vector3d ToDegrees(this Vector3d radians)
        {
            return new Vector3d(
                FixedMath.Rad2Deg * radians.x,
                FixedMath.Rad2Deg * radians.y,
                FixedMath.Rad2Deg * radians.z);
        }

        /// <summary>
        /// Converts each component of the vector from degrees to radians.
        /// </summary>
        /// <param name="degrees">The vector with components in degrees.</param>
        /// <returns>A new vector with components converted to radians.</returns>
        public static Vector3d ToRadians(this Vector3d degrees)
        {
            return new Vector3d(
                FixedMath.Deg2Rad * degrees.x,
                FixedMath.Deg2Rad * degrees.y,
                FixedMath.Deg2Rad * degrees.z);
        }

        /// <summary>
        /// Compares two vectors for approximate equality, allowing an absolute epsilon difference.
        /// </summary>
        public static bool FuzzyEqual(this Vector3d me, Vector3d other)
        {
            return me.FuzzyEqualAbsolute(other, FixedMath.Epsilon);
        }

        /// <summary>
        /// Compares two vectors for approximate equality, allowing a fixed absolute difference.
        /// </summary>
        /// <param name="me">The current vector.</param>
        /// <param name="other">The vector to compare against.</param>
        /// <param name="allowedDifference">The allowed absolute difference between each component.</param>
        /// <returns>True if the components are within the allowed difference, false otherwise.</returns>
        public static bool FuzzyEqualAbsolute(this Vector3d me, Vector3d other, Fixed64 allowedDifference)
        {
            var dx = me.x - other.x;
            if (dx.Abs() > allowedDifference)
                return false;

            var dy = me.y - other.y;
            if (dy.Abs() > allowedDifference)
                return false;

            var dz = me.z - other.z;
            return dz.Abs() < allowedDifference;
        }

        /// <summary>
        /// Compares two vectors for approximate equality, allowing a fractional difference (percentage).
        /// Handles zero components by only using the allowed percentage difference.
        /// </summary>
        /// <param name="me">The current vector.</param>
        /// <param name="other">The vector to compare against.</param>
        /// <param name="percentage">The allowed fractional difference (percentage) for each component.</param>
        /// <returns>True if the components are within the allowed percentage difference, false otherwise.</returns>
        public static bool FuzzyEqual(this Vector3d me, Vector3d other, Fixed64 percentage)
        {
            var dx = me.x - other.x;
            if (dx.Abs() > (me.x == FixedMath.Zero ? percentage : me.x * percentage))
                return false;

            var dy = me.y - other.y;
            if (dy.Abs() > (me.y == FixedMath.Zero ? percentage : me.y * percentage))
                return false;

            var dz = me.z - other.z;
            return dz.Abs() < (me.z == FixedMath.Zero ? percentage : me.z * percentage);
        }

        /// <summary>
        /// Checks if the distance between two vectors is less than or equal to a specified factor.
        /// </summary>
        /// <param name="me">The current vector.</param>
        /// <param name="other">The vector to compare distance to.</param>
        /// <param name="factor">The maximum allowable distance.</param>
        /// <returns>True if the distance between the vectors is less than or equal to the factor, false otherwise.</returns>
        public static bool CheckDistance(this Vector3d me, Vector3d other, Fixed64 factor)
        {
            var dis = Vector3d.Distance(me, other);
            if (dis <= factor)
                return true;
            else
                return false;
        }

        /// <summary>
        /// Calculates the squared distance between two vectors, avoiding the need for a square root operation.
        /// </summary>
        /// <param name="vec">The current vector.</param>
        /// <param name="other">The vector to calculate the squared distance to.</param>
        /// <returns>The squared distance between the two vectors.</returns>
        public static Fixed64 V3SqrDistance(this Vector3d vec, Vector3d other)
        {
            Fixed64 temp1 = vec.x - other.x;
            temp1 *= temp1;
            Fixed64 temp2 = vec.y - other.y;  // Corrected subtraction
            temp2 *= temp2;
            Fixed64 temp3 = vec.z - other.z;
            temp3 *= temp3;
            return temp1 + temp2 + temp3;
        }

        /// <summary>
        /// Rotates the vector around a given position using a specified quaternion rotation.
        /// </summary>
        /// <param name="source">The vector to rotate.</param>
        /// <param name="position">The position around which the vector is rotated.</param>
        /// <param name="rotation">The quaternion representing the rotation.</param>
        /// <returns>The rotated vector.</returns>
        public static Vector3d RotateVector(this Vector3d source, Vector3d position, FixedQuaternion rotation)
        {
            source -= position;
            return (rotation * source) + position;
        }

        /// <summary>
        /// Applies the inverse of a specified quaternion rotation to the vector around a given position.
        /// </summary>
        /// <param name="source">The vector to rotate.</param>
        /// <param name="position">The position around which the vector is rotated.</param>
        /// <param name="rotation">The quaternion representing the inverse rotation.</param>
        /// <returns>The rotated vector.</returns>
        public static Vector3d InverseRotateVector(this Vector3d source, Vector3d position, FixedQuaternion rotation)
        {
            // Subtract the position
            source -= position;
            // Undo the rotation
            source = rotation.Inverse() * source;
            // Add the original position back
            return source + position;
        }

        /// <summary>
        /// Sets the global scale of an object using a 4x4 transformation matrix.
        /// </summary>
        /// <param name="matrix">The transformation matrix representing the object's global state.</param>
        /// <param name="globalScale">The desired global scale as a vector.</param>
        /// <remarks>
        /// The method extracts the current global scale from the matrix and computes the new local scale 
        /// by dividing the desired global scale by the current global scale. 
        /// The new local scale is then applied to the matrix.
        /// </remarks>
        public static void SetGlobalScale(this ref FixedMatrix4x4 matrix, Vector3d globalScale)
        {
            // Reset the local scaling portion of the matrix
            matrix.SetIdentityScale();

            // Extract the current global (lossy) scale from the matrix
            Vector3d currentGlobalScale = matrix.ExtractScale();

            // Compute the new local scale by dividing the desired global scale by the current global scale
            Vector3d newLocalScale = new Vector3d(
                globalScale.x / currentGlobalScale.x,
                globalScale.y / currentGlobalScale.y,
                globalScale.z / currentGlobalScale.z
            );

            // Apply the new local scale to the matrix
            matrix.ApplyScale(newLocalScale);
        }

        /// <summary>
        /// Sets the global scale of an object using FixedMatrix3x3.
        /// Similar to SetGlobalScale for FixedMatrix4x4, but for a 3x3 matrix.
        /// </summary>
        /// <param name="matrix">The transformation matrix (3x3) representing the object's global state.</param>
        /// <param name="globalScale">The desired global scale represented as a Vector3d.</param>
        /// <remarks>
        /// The method extracts the current global scale from the matrix and computes the new local scale 
        /// by dividing the desired global scale by the current global scale. 
        /// The new local scale is then applied to the matrix.
        /// </remarks>
        public static void SetGlobalScale(this ref FixedMatrix3x3 matrix, Vector3d globalScale)
        {
            // Reset the local scaling portion of the matrix
            matrix.SetIdentityScale();

            // Extract the current global (lossy) scale from the matrix
            Vector3d currentGlobalScale = matrix.ExtractScale();

            // Compute the new local scale by dividing the desired global scale by the current global scale
            Vector3d newLocalScale = new Vector3d(
                globalScale.x / currentGlobalScale.x,
                globalScale.y / currentGlobalScale.y,
                globalScale.z / currentGlobalScale.z
            );

            // Apply the new local scale to the matrix
            matrix.ApplyScale(newLocalScale);
        }
    }
}
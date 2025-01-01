#if UNITY_EDITOR
using UnityEditor;
using UnityEngine;

namespace FixedMathSharp.Editor
{
    [CustomPropertyDrawer(typeof(VectorRotationAttribute))]
    public class VectorRotationDrawer : PropertyDrawer
    {
        private float height = 0f;

        public override float GetPropertyHeight(SerializedProperty property, GUIContent label)
        {
            return height;
        }

        public override void OnGUI(Rect position, SerializedProperty property, GUIContent label)
        {
            EditorGUI.BeginProperty(position, label, property);

            // Begin tracking changes
            EditorGUI.BeginChangeCheck();

            VectorRotationAttribute at = (VectorRotationAttribute)attribute;

            SerializedProperty x = property.FindPropertyRelative("x");
            SerializedProperty y = property.FindPropertyRelative("y");
            if(x.GetFixedPropertyValue() is Fixed64 FixedX && y.GetFixedPropertyValue() is Fixed64 FixedY)
            {
                // Calculate the angle in radians and degrees
                Fixed64 angleInRadians = FixedMath.Atan2(FixedY, FixedX);
                Fixed64 angleInDegrees = FixedMath.RadToDeg(angleInRadians) * (int)at.Timescale;

                height = 15f;
                position.height = height;

                // Draw the angle in degrees in the inspector
                FixedMathEditorUtility.DoubleField(position, "Angle", ref angleInDegrees, at.Timescale);

                // Convert the new angle back to radians
                Fixed64 newAngleInRadians = FixedMath.DegToRad(angleInDegrees);

                // Check if the change in angle is significant enough to update
                if ((newAngleInRadians - angleInRadians).Abs() >= new Fixed64(.001f))
                {
                    Fixed64 cos = FixedMath.Cos(newAngleInRadians);
                    Fixed64 sin = FixedMath.Sin(newAngleInRadians);

                    // Update the vector components based on the new angle
                    property.SetFixedPropertyValue(new Vector2d(cos, sin));
                }

                // Apply changes only if something has been modified
                if (EditorGUI.EndChangeCheck())
                {
                    property.serializedObject.ApplyModifiedProperties();
                }
            }
            else
            {
                Debug.LogWarning("Property value for x|y is null or not a Fixed64.");
            }

            EditorGUI.EndProperty();
        }
    }
}
#endif
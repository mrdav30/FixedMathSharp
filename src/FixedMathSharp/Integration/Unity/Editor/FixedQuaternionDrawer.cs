#if UNITY_EDITOR
using UnityEditor;
using UnityEngine;

namespace FixedMathSharp.Editor
{
    [CustomPropertyDrawer(typeof(FixedQuaternion)), CanEditMultipleObjects]
    public class FixedQuaternionDrawer : PropertyDrawer
    {
        public override void OnGUI(Rect position, SerializedProperty property, GUIContent label)
        {
            EditorGUI.BeginProperty(position, label, property);

            // Draw label
            Rect contentPosition = EditorGUI.PrefixLabel(position, label);

            // Get the FixedQuaternion instance from the serialized property
            if (property.GetFixedPropertyValue() is FixedQuaternion quaternion)
            {
                // Convert the quaternion to euler angles (in degrees)
                Vector3d eulerAngles = quaternion.EulerAngles;

                using var indent = new EditorGUI.IndentLevelScope(-EditorGUI.indentLevel);
                // Display euler angles in the inspector and get the new values after editing
                FixedMathEditorUtility.Vector3dField(contentPosition, GUIContent.none, ref eulerAngles);

                // Convert the edited euler angles back to a quaternion (in radians) and set the quaternion value
                FixedQuaternion newQuaternion = FixedQuaternion.FromEulerAnglesInDegrees(eulerAngles.x, eulerAngles.y, eulerAngles.z);

                if (EditorGUI.EndChangeCheck() && !Application.isPlaying)
                {
                    SerializedProperty x = property.FindPropertyRelative("x").FindPropertyRelative("m_rawValue");
                    x.longValue = newQuaternion.x.m_rawValue;
                    SerializedProperty y = property.FindPropertyRelative("y").FindPropertyRelative("m_rawValue");
                    y.longValue = newQuaternion.y.m_rawValue;
                    SerializedProperty z = property.FindPropertyRelative("z").FindPropertyRelative("m_rawValue");
                    z.longValue = newQuaternion.z.m_rawValue;
                    SerializedProperty w = property.FindPropertyRelative("w").FindPropertyRelative("m_rawValue");
                    w.longValue = newQuaternion.w.m_rawValue;
                }
            }
            else
            {
                Debug.LogWarning("Property value is null or not a FixedQuaternion.");
            }      

            EditorGUI.EndProperty();
        }
    }
}
#endif
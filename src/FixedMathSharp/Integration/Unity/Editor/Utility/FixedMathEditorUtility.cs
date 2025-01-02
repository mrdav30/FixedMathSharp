#if UNITY_EDITOR
using System;
using UnityEditor;
using UnityEngine;

namespace FixedMathSharp.Editor
{
    public static class FixedMathEditorUtility
    {
        //public static double Scale(bool isScaled)
        //{
        //    return isScaled ? LockstepManager.FrameRate : 1;
        //}

        public static void DoubleField(Rect position, GUIContent label, ref Fixed64 value, double scale = 1d)
        {
            value = (Fixed64)(EditorGUI.DoubleField(position, label, (double)value * scale) / scale);
        }

        public static void DoubleField(Rect position, string label, ref Fixed64 value, double scale = 1d)
        {
            value = (Fixed64)(EditorGUI.DoubleField(position, label, (double)value * scale) / scale);
        }

        public static Fixed64 FixedNumberField(Rect position, Fixed64 value, Fixed64 max)
        {
            Fixed64 result = (Fixed64)EditorGUI.DoubleField(position, Math.Round((double)value, 2, MidpointRounding.AwayFromZero));
            return max == Fixed64.Zero || result <= max ? result : max;
        }

        public static Fixed64 FixedNumberField(Rect position, Fixed64 value)
        {
            Fixed64 result = (Fixed64)EditorGUI.DoubleField(position, Math.Round((double)value, 2, MidpointRounding.AwayFromZero));
            return result;
        }

        public static Fixed64 FixedNumberField(Rect position, long value)
        {
            return FixedNumberField(position, GUIContent.none, value);
        }

        public static Fixed64 FixedNumberField(Rect position, GUIContent label, long value)
        {
            return (Fixed64)EditorGUI.DoubleField(position, label, Math.Round(Fixed64.ToDouble(value), 4, MidpointRounding.AwayFromZero));
        }

        public static void FixedNumberField(string Label, ref Fixed64 fixedNumber)
        {
            fixedNumber = (Fixed64)EditorGUILayout.DoubleField(Label, (double)fixedNumber);
        }

        public static void Vector2dField(string Label, ref Vector2d vector)
        {
            vector = EditorGUILayout.Vector2Field(Label, vector.ToVector2()).ToVector2d();
        }

        public static void Vector3dField(string Label, ref Vector3d vector)
        {
            vector = EditorGUILayout.Vector3Field(Label, vector.ToVector3()).ToVector3d();
        }

        public static void Vector3dField(Rect position, GUIContent label, ref Vector3d vector)
        {
            float labelWidth = EditorGUIUtility.labelWidth;
            EditorGUIUtility.labelWidth = 14f;

            float fieldWidth = position.width / 3f;
            Rect fieldRect = new Rect(position.x, position.y, position.width / 3.5f, position.height);

            EditorGUI.BeginChangeCheck();
            Fixed64 x = (Fixed64)EditorGUI.DoubleField(fieldRect, "X", vector.x.ToFormattedDouble(3));
            fieldRect.x += fieldWidth;
            Fixed64 y = (Fixed64)EditorGUI.DoubleField(fieldRect, "Y", vector.y.ToFormattedDouble(3));
            fieldRect.x += fieldWidth;
            Fixed64 z = (Fixed64)EditorGUI.DoubleField(fieldRect, "Z", vector.z.ToFormattedDouble(3));

            if (EditorGUI.EndChangeCheck())
            {
                vector = new Vector3d(x, y, z);
            }

            EditorGUIUtility.labelWidth = labelWidth;
        }
    }
}
#endif
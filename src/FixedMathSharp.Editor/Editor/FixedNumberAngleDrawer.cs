#if UNITY_EDITOR
using UnityEditor;
using UnityEngine;

namespace FixedMathSharp.Editor
{
    [CustomPropertyDrawer(typeof(FixedNumberAngleAttribute))]
	public class FixedNumberAngleDrawer : PropertyDrawer
	{
		public override void OnGUI(Rect position, SerializedProperty property, GUIContent label)
		{
			FixedNumberAngleAttribute angleAttribute = (FixedNumberAngleAttribute)attribute;
			Fixed64 value = new Fixed64(property.longValue);

			// Calculate the angle, rounding to 2 decimal places
			Fixed64 angle = FixedMath.RoundToPrecision(FixedMath.RadToDeg(FixedMath.Asin(value)), 2);

			FixedMathEditorUtility.DoubleField(position, label, ref angle, angleAttribute.Timescale);

			// Check if the max value is valid, and clamp the angle if necessary
			Fixed64 max = new Fixed64(angleAttribute.Max);
			if (max > Fixed64.Zero && angle > max)
				angle = max;

			property.longValue = FixedMath.Sin(FixedMath.DegToRad(angle)).m_rawValue;
		}
	}
}
#endif
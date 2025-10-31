#if InputSystemIsPresent && UNITY_2022_1_OR_NEWER

using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.InputSystem.Processors;

namespace AC.Templates.InputSystemIntegration
{

	public class ScaleProcessorLink : MonoBehaviour
	{

		#region Variables

		[SerializeField] private string actionName = "CursorHorizontal";
		[SerializeField] private string globalStringVariableName = "Sensitivity";
		[SerializeField] private UnityEngine.InputSystem.PlayerInput playerInput;
		private GVar globalStringVariable;
		
		#endregion


		#region UnityStandards

		private void Update ()
		{
			if (globalStringVariable == null)
			{
				globalStringVariable = GlobalVariables.GetVariable (globalStringVariableName);
			}
			if (globalStringVariable != null)
			{
				var action = playerInput.actions.FindAction (actionName);
				action.ApplyParameterOverride ((ScaleProcessor p) => p.factor, globalStringVariable.FloatValue);
			}
		}

		#endregion

	}

}

#endif
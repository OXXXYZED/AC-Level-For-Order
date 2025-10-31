#if UNITY_2022_1_OR_NEWER

using UnityEngine;
#if InputSystemIsPresent
using UnityEngine.InputSystem;
#endif

namespace AC.Templates.InputSystemIntegration
{

	public class InputRemapper : MonoBehaviour
	{

#if InputSystemIsPresent

		#region Variables

		[SerializeField] private RemappableAction[] remappableActions = new RemappableAction[0];
		private RemappableAction activeRebinding;

		#endregion


		#region UnityStandards

		private void Start ()
		{
			foreach (RemappableAction remappableAction in remappableActions)
			{
				remappableAction.AddClickEvent (() => OnClickRemapButton (remappableAction));
			}
		}


		private void OnEnable ()
		{
			foreach (RemappableAction remappableAction in remappableActions)
			{
				remappableAction.EndRebinding ();

				// Because Input System now clones the input actions asset at runtime, need to override the Actions
				if (ControlsReader.Instance)
				{
					var inputActionAsset = ControlsReader.Instance.PlayerInput.actions;
					remappableAction.AssignOverride (inputActionAsset);
				}
			}
		}


		private void OnDisable ()
		{
			foreach (RemappableAction remappableAction in remappableActions)
			{
				remappableAction.EndRebinding ();
			}
		}

		#endregion


		#region PublicFunctions

#if UNITY_EDITOR

		public void SwapInputActionAsset (InputActionAsset inputActionAsset)
		{
			var serializedObject = new UnityEditor.SerializedObject (this);
			var property = serializedObject.FindProperty("remappableActions");

			for (int i = 0; i < property.arraySize; i++)
			{
				var element = property.GetArrayElementAtIndex(i);
				var field = element.FindPropertyRelative ("inputActionReference");

				var newReference = remappableActions[i].GetNewInputActionReference (inputActionAsset);
				if (newReference)
				{
					field.boxedValue = newReference;
				}
				else
				{
					Debug.LogWarning ("Could not find action reference for " + field.boxedValue);
				}
			}
			serializedObject.ApplyModifiedProperties ();
			UnityVersionHandler.CustomSetDirty (this);
		}
#endif


		public void OnClickRemapButton (RemappableAction remappableAction)
		{
			if (activeRebinding != null)
			{
				activeRebinding.EndRebinding ();

				if (remappableAction == activeRebinding)
				{
					activeRebinding = null;
					return;
				}
			}

			activeRebinding = remappableAction;
			activeRebinding.BeginRebinding ();
		}


		public void OnClickDefaultsButton ()
		{
			ControlsReader controlsReader = ControlsReader.Instance;
			if (controlsReader == null) return;

			if (activeRebinding != null)
			{
				activeRebinding.EndRebinding ();
				activeRebinding = null;
			}
			
			controlsReader.ClearRebindings ();

			foreach (RemappableAction remappableAction in remappableActions)
			{
				remappableAction.EndRebinding ();
			}
		}

		#endregion

#endif

	}

}

#endif
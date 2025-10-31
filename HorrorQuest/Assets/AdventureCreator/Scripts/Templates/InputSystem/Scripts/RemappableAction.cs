#if InputSystemIsPresent && UNITY_2022_1_OR_NEWER

using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Events;
using UnityEngine.InputSystem;
using UnityEngine.UI;

#if UNITY_EDITOR
using UnityEditor;
#endif

namespace AC.Templates.InputSystemIntegration
{

	[Serializable]
	public class RemappableAction
	{

		#region Variables

		[SerializeField] private InputActionReference inputActionReference = null;
		[SerializeField] private UnityEngine.UI.Button linkedUIButton = null;
		private InputActionRebindingExtensions.RebindingOperation activeRebindingOperation;
		private InputAction overrideAction;

		#endregion


		#region PublicFunctions

#if UNITY_EDITOR

		public InputActionReference GetNewInputActionReference (InputActionAsset inputActionAsset)
		{
			if (inputActionAsset == null || inputActionReference == null || inputActionReference.action == null) return null;

			UnityEngine.Object[] subAssets = UnityEditor.AssetDatabase.LoadAllAssetsAtPath (UnityEditor.AssetDatabase.GetAssetPath (inputActionAsset));
			foreach (UnityEngine.Object obj in subAssets)
			{
				if (obj is InputActionReference)
				{
					InputActionReference _inputActionReference = (InputActionReference) obj;
					if ((_inputActionReference.hideFlags & HideFlags.HideInHierarchy) == 0)
					{
						if (_inputActionReference.action != null && _inputActionReference.action.name == inputActionReference.action.name)
						{
							return _inputActionReference;
						}
					}
				}
			}
			return null;
		}
		
#endif

		public void AssignOverride (InputActionAsset asset)
		{
			var action = asset.FindAction (inputActionReference.action.name);
			if (action != null)
			{
				overrideAction = action;
			}
		}



		public void AddClickEvent (UnityAction onClick)
		{
			linkedUIButton.onClick.AddListener (onClick);
		}


		public void BeginRebinding ()
		{
			Action.Disable ();

			activeRebindingOperation = Action.PerformInteractiveRebinding ()
				.WithControlsExcluding ("Mouse")
				.OnMatchWaitForAnother (0.1f)
				.OnComplete (operation => OnRemapBinding ())
				.Start ();

			SetTextToPending ();
		}


		public void EndRebinding ()
		{
			if (activeRebindingOperation != null)
			{
				activeRebindingOperation.Dispose ();
				Action.Enable ();
				activeRebindingOperation = null;
			}

			SetTextToLabel ();
		}

		#endregion


		#region PrivateFunctions

		private void OnRemapBinding ()
		{
			EndRebinding ();

			if (ControlsReader.Instance)
			{
				/*var bindings = inputActionReference.ToInputAction ().bindings;
				for (int j = 0; j < bindings.Count; j++)
				{
					var overridePath = bindings[j].overridePath;
					if (!string.IsNullOrEmpty (overridePath))
					{
						ControlsReader.Instance.SaveRebindings (inputActionReference.name, overridePath);
					}
				}*/
				ControlsReader.Instance.SaveRebindings ();
			}
			else
			{
				ACDebug.LogWarning ("Cannot complete rebinding because no Controls Reader component was found");
			}
		}


		private void SetTextToLabel ()
		{
			if (inputActionReference == null) return;

			int bindingIndex = Action.GetBindingIndexForControl (Action.controls[0]);
			string inputKey = InputControlPath.ToHumanReadableString (Action.bindings[bindingIndex].effectivePath, InputControlPath.HumanReadableStringOptions.OmitDevice);

			SetButtonText (Action.name + " - " + inputKey);
		}


		private void SetTextToPending ()
		{
			if (inputActionReference == null) return;

			SetButtonText (Action.name + " - " + "(Press a key)");
		}


		private void SetButtonText (string text)
		{
			if (linkedUIButton == null) return;
			#if TextMeshProIsPresent
			TMPro.TextMeshProUGUI textBox = linkedUIButton.GetComponentInChildren<TMPro.TextMeshProUGUI> ();
			#else
			Text textBox = linkedUIButton.GetComponentInChildren<Text> ();
			#endif
			if (textBox == null) return;

			textBox.text = text;
		}

		#endregion


		#region GetSet

		public InputAction Action
		{
			get
			{
				if (overrideAction != null)
				{
					return overrideAction;
				}
				if (inputActionReference != null)
				{
					return inputActionReference.action;
				}
				return null;
			}
		}

		#endregion

	}

}

#endif
#if UNITY_2022_1_OR_NEWER

using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.Events;
using UnityEngine.EventSystems;
#if InputSystemIsPresent
using UnityEngine.InputSystem;
using UnityEngine.InputSystem.UI;
#endif

namespace AC.Templates.InputSystemIntegration
{

	public class ControlsReader : MonoBehaviour
	{

#if InputSystemIsPresent

		#region Variables

		[SerializeField] private UnityEngine.InputSystem.PlayerInput playerInput = null;
		[SerializeField] private InputSystemUIInputModule uiInputModule = null;
		[SerializeField] private string globalStringVariable = "InputRebindings";
		private readonly Dictionary<string, InputAction> inputActionsDictionary = new Dictionary<string, InputAction> ();
		private string currentControlSchemeName;
		public StringEvent onSetControlScheme;
		[Serializable] public class StringEvent : UnityEvent<string> { }

		[SerializeField] private bool useEnhancedTouchSupport;
		[SerializeField] private bool autoSyncInputMethod = true;
		[SerializeField] private bool useSimulatedCursorForUIEvents = true;
		[SerializeField] private bool autoDisableUIClicks = true;
		[SerializeField] private ControlSchemeLink[] controlSchemeLinks = new ControlSchemeLink[0];
		[SerializeField] private bool suppressMissingInputWarnings;

		private Vector2 lastCursorPosition;
		private List<RaycastResult> raycastResults = new List<RaycastResult>();

		#endregion


		#region UnityStandards

		private void Start ()
		{
			if (playerInput == null)
			{
				playerInput = GetComponent<UnityEngine.InputSystem.PlayerInput> ();
				if (playerInput == null)
				{
					ACDebug.LogWarning ("AC's Controls Reader component requires Unity's Player Input component to be assigned", this);
					return;
				}
			}

			SetDelegates ();
		}


		private void OnEnable ()
		{
			if (useEnhancedTouchSupport)
			{
				UnityEngine.InputSystem.EnhancedTouch.EnhancedTouchSupport.Enable ();
			}

			inputActionsDictionary.Clear ();
			playerInput.actions.Enable ();
			EventManager.OnInitialiseScene += OnInitialiseScene;
			EventManager.OnSwitchProfile += OnSwitchProfile;
			SetDelegates ();
		}


		private void OnDisable ()
		{
			if (useEnhancedTouchSupport)
			{
				UnityEngine.InputSystem.EnhancedTouch.EnhancedTouchSupport.Disable ();
			}

			playerInput.actions.Disable ();
			EventManager.OnSwitchProfile -= OnSwitchProfile;
			EventManager.OnInitialiseScene -= OnInitialiseScene;
		}


		private void OnDestroy ()
		{
			ClearActionEvents ();
		}


		private void Update ()
		{
			if (autoDisableUIClicks && uiInputModule)
			{
				if (CanDirectlyControlMenus ())
				{
					if (uiInputModule.leftClick) uiInputModule.leftClick.action.Disable ();
					if (uiInputModule.rightClick) uiInputModule.rightClick.action.Disable ();
				}
				else
				{
					if (uiInputModule.leftClick) uiInputModule.leftClick.action.Enable ();
					if (uiInputModule.rightClick) uiInputModule.rightClick.action.Enable ();
				}
			}
			
			if (!useSimulatedCursorForUIEvents) return;

			Vector2 simulatedCursorPosition = KickStarter.playerInput.GetMousePosition ();
			var pointerEvent = new PointerEventData (EventSystem.current)
			{
				position = simulatedCursorPosition,
				delta = simulatedCursorPosition - lastCursorPosition,
				pointerId = -1,
				//pointerCurrentRaycast = new RaycastResult ()
			};

			if (EventSystem.current)
			{
				EventSystem.current.RaycastAll (pointerEvent, raycastResults);
				GameObject obToSelect = raycastResults.Count > 0 ? raycastResults[0].gameObject : null;
				EventSystem.current.SetSelectedGameObject(obToSelect);
			}

			lastCursorPosition = simulatedCursorPosition;
		}

		#endregion


		#region MouseInput

		protected virtual Vector2 Custom_MousePosition (bool cursorIsLocked)
		{
			if (cursorIsLocked)
				return new Vector2 (Screen.width / 2f, Screen.height / 2f);

			return Mouse.current.position.ReadValue ();
		}


		protected virtual bool Custom_GetMouseButton (int button)
		{
			if (!KickStarter.settingsManager.defaultMouseClicks)
			{
				return false;
			}

			switch (button)
			{
				case 0:
					return Mouse.current.leftButton.isPressed;

				case 1:
					return Mouse.current.rightButton.isPressed;

				default:
					return false;
			}
		}


		protected virtual bool Custom_GetMouseButtonDown (int button)
		{
			if (!KickStarter.settingsManager.defaultMouseClicks)
			{
				return false;
			}

			switch (button)
			{
				case 0:
					return Mouse.current.leftButton.wasPressedThisFrame;

				case 1:
					return Mouse.current.rightButton.wasPressedThisFrame;

				default:
					return false;
			}
		}


		protected virtual Vector2 Custom_GetFreeAim (bool cursorIsLocked)
		{
			if (cursorIsLocked)
			{
				return new Vector2 (Custom_GetAxis ("CursorHorizontal"), Custom_GetAxis ("CursorVertical"));
			}
			return Vector2.zero;
		}

		#endregion


		#region KeyboardControllerInput

		protected virtual float Custom_GetAxis (string axisName)
		{
			InputAction inputAction = GetInputAction (axisName);
			if (inputAction == null)
			{
				return 0f;
			}

			return inputAction.ReadValue<float> ();
		}


		protected virtual bool Custom_GetButton (string axisName)
		{
			InputAction inputAction = GetInputAction (axisName);
			if (inputAction == null)
			{
				return false;
			}

			return inputAction.IsPressed ();
		}


		protected virtual bool Custom_GetButtonDown (string axisName)
		{
			InputAction inputAction = GetInputAction (axisName);
			if (inputAction == null)
			{
				return false;
			}

			return inputAction.WasPerformedThisFrame ();
		}


		protected virtual bool Custom_GetButtonUp (string axisName)
		{
			InputAction inputAction = GetInputAction (axisName);
			if (inputAction == null)
			{
				return false;
			}

			return inputAction.WasReleasedThisFrame ();
		}

		#endregion


		#region PublicFunctions

		public void ClearRebindings ()
		{
			GVar saveVariable = GlobalVariables.GetVariable (globalStringVariable);
			if (saveVariable != null && saveVariable.type == VariableType.String)
			{
				saveVariable.TextValue = string.Empty;
				Options.SavePrefs ();
			}

			LoadRebindings ();
		}


		public void SaveRebindings ()
		{
			GVar saveVariable = GlobalVariables.GetVariable (globalStringVariable);
			if (saveVariable != null && saveVariable.type == VariableType.String)
			{
				saveVariable.TextValue = playerInput.actions.SaveBindingOverridesAsJson ();
				Options.SavePrefs ();
			}
			else
			{
				ACDebug.LogWarning ("Cannot find a Global String Variable to store input rebinding data in", this);
			}
		}

		#endregion


		#region CustomEvents

		private void OnInitialiseScene ()
		{
			SetDelegates ();
			LoadRebindings ();
		}


		private void OnSwitchProfile (int profileID)
		{
			LoadRebindings ();
		}

		#endregion


		#region PrivateFunctions

		private void SetDelegates ()
		{
#if !UNITY_EDITOR
			if (KickStarter.settingsManager.inputMethod == InputMethod.TouchScreen)
			{
				UnityEngine.InputSystem.EnhancedTouch.EnhancedTouchSupport.Enable ();
			}
#endif
			if (KickStarter.playerInput == null) return;

			// Mouse delegates
			KickStarter.playerInput.InputMousePositionDelegate = Custom_MousePosition;
			KickStarter.playerInput.InputGetMouseButtonDelegate = Custom_GetMouseButton;
			KickStarter.playerInput.InputGetMouseButtonDownDelegate = Custom_GetMouseButtonDown;
			KickStarter.playerInput.InputGetFreeAimDelegate = Custom_GetFreeAim;

			// Keyboard / controller delegates
			KickStarter.playerInput.InputGetAxisDelegate = Custom_GetAxis;
			KickStarter.playerInput.InputGetButtonDelegate = Custom_GetButton;
			KickStarter.playerInput.InputGetButtonDownDelegate = Custom_GetButtonDown;
			KickStarter.playerInput.InputGetButtonUpDelegate = Custom_GetButtonUp;

			// Touch delegates
			KickStarter.playerInput.InputTouchCountDelegate = Custom_TouchCount;
			KickStarter.playerInput.InputTouchPositionDelegate = Custom_TouchPosition;
			KickStarter.playerInput.InputTouchDeltaPositionDelegate = Custom_TouchDeltaPosition;
			KickStarter.playerInput.InputGetTouchPhaseDelegate = Custom_TouchPhase;
		}


		private void LoadRebindings ()
		{
			GVar saveVariable = GlobalVariables.GetVariable (globalStringVariable);
			if (saveVariable != null && saveVariable.type == VariableType.String)
			{
				if (!string.IsNullOrEmpty (saveVariable.TextValue))
				{
					playerInput.actions.LoadBindingOverridesFromJson (saveVariable.TextValue);
				}
				else
				{
					foreach (var action in playerInput.actions)
					{
						action.RemoveAllBindingOverrides ();
					}
				}
			}
			else
			{
				ACDebug.LogWarning ("Cannot find a Global String Variable to store input rebinding data in", this);
			}
		}


		/*private void CopyOverridenBindings (InputActionAsset inputActionAsset, string actionName, string _overridePath)
		{
			var action = inputActionAsset.FindAction (actionName);
			if (action != null)
			{
				action.ApplyBindingOverride(new InputBinding { overridePath = _overridePath });
			}
		}*/


		public InputAction GetInputAction (string axisName)
		{
			if (inputActionsDictionary.TryGetValue (axisName, out InputAction inputAction))
			{
				return inputAction;
			}

			inputAction = playerInput.actions.FindAction (axisName);
			if (inputAction != null)
			{
				inputActionsDictionary.Add (axisName, inputAction);
				inputAction.performed += OnInputActionPerformed;
				return inputAction;
			}

			if (!suppressMissingInputWarnings)
			{
				ACDebug.LogWarning ("Input Action '" + axisName + "' not found", this);
			}
			inputActionsDictionary.Add (axisName, inputAction);
			return null;
		}


		private void OnInputActionPerformed (InputAction.CallbackContext callbackContext)
		{
			InputDevice[] devices = new InputDevice[1] { callbackContext.control.device };
			var controlScheme = InputControlScheme.FindControlSchemeForDevices (devices, playerInput.actions.controlSchemes);
			controlScheme = playerInput.actions.controlSchemes.First (x => x.SupportsDevice (devices[0])); // FindControlSchemeForDevices returns null for optional keyboard

			if (!controlScheme.HasValue)
			{
				return;
			}

			SetCurrentDevice (controlScheme.Value.name);
		}


		private void SetCurrentDevice (string controlScheme)
		{
			if (currentControlSchemeName == controlScheme || string.IsNullOrEmpty (controlScheme))
			{
				return;
			}

			currentControlSchemeName = controlScheme;
			if (onSetControlScheme != null)
			{
				onSetControlScheme.Invoke (currentControlSchemeName);
			}

			if (autoSyncInputMethod)
			{
				foreach (ControlSchemeLink controlSchemeLink in controlSchemeLinks)
				{
					if (controlSchemeLink.controlScheme == currentControlSchemeName)
					{
						KickStarter.settingsManager.inputMethod = controlSchemeLink.linkedInputMethod;
						return;
					}
				}
			}
		}


		private void ClearActionEvents ()
		{
			foreach (InputAction inputAction in inputActionsDictionary.Values)
			{
				if (inputAction != null)
				{
					inputAction.performed -= OnInputActionPerformed;
				}
			}
		}


		private bool CanDirectlyControlMenus ()
		{
			if (KickStarter.stateHandler == null || KickStarter.settingsManager == null) return false;
			if (KickStarter.settingsManager.inputMethod == InputMethod.TouchScreen) return false;

			if ((KickStarter.stateHandler.gameState == GameState.Paused && KickStarter.menuManager.keyboardControlWhenPaused) ||
				(KickStarter.stateHandler.gameState == GameState.DialogOptions && KickStarter.menuManager.keyboardControlWhenDialogOptions) ||
				(KickStarter.stateHandler.IsInGameplay () && KickStarter.playerInput.canKeyboardControlMenusDuringGameplay))
			{
				return true;
			}
			return false;
		}

		#endregion


		#region TouchInput

		protected virtual int Custom_TouchCount ()
		{
			return UnityEngine.InputSystem.EnhancedTouch.Touch.activeTouches.Count;
		}


		protected virtual Vector2 Custom_TouchPosition (int index)
		{
			return UnityEngine.InputSystem.EnhancedTouch.Touch.activeTouches[index].screenPosition;
		}


		protected virtual Vector2 Custom_TouchDeltaPosition (int index)
		{
			return UnityEngine.InputSystem.EnhancedTouch.Touch.activeTouches[index].delta;
		}


		protected virtual UnityEngine.TouchPhase Custom_TouchPhase (int index)
		{
			UnityEngine.InputSystem.TouchPhase touchPhase = UnityEngine.InputSystem.EnhancedTouch.Touch.activeTouches[index].phase;
			switch (touchPhase)
			{
				case UnityEngine.InputSystem.TouchPhase.Began:
					return UnityEngine.TouchPhase.Began;

				case UnityEngine.InputSystem.TouchPhase.Canceled:
					return UnityEngine.TouchPhase.Canceled;

				case UnityEngine.InputSystem.TouchPhase.Ended:
					return UnityEngine.TouchPhase.Ended;

				case UnityEngine.InputSystem.TouchPhase.Moved:
					return UnityEngine.TouchPhase.Moved;

				case UnityEngine.InputSystem.TouchPhase.Stationary:
					return UnityEngine.TouchPhase.Stationary;

				default:
					return UnityEngine.TouchPhase.Canceled;
			}
		}

		#endregion


		#region GetSet

		public static ControlsReader Instance
		{
			get
			{
#if UNITY_2022_3_OR_NEWER
				ControlsReader controlsReader = UnityEngine.Object.FindFirstObjectByType <ControlsReader> ();
#else
				ControlsReader controlsReader = UnityEngine.Object.FindObjectOfType <ControlsReader> ();
#endif
				if (controlsReader == null)
				{
					ACDebug.LogWarning ("Cannot find ControlsReader component");
				}
				return controlsReader;
			}
		}


		public string CurrentControlSchemeName { get { return currentControlSchemeName; }}
		public string GlobalStringVariable { get { return globalStringVariable; }}
		public UnityEngine.InputSystem.PlayerInput PlayerInput => playerInput;

		#endregion


		#region PrivateClasses

		[Serializable]
		private class ControlSchemeLink
		{

			public string controlScheme;
			public InputMethod linkedInputMethod;

		}

		#endregion

#endif

	}

}

#endif
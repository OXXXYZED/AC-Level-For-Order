#if InputSystemIsPresent && UNITY_2022_1_OR_NEWER

#if UNITY_EDITOR
using UnityEditor;
#endif
using UnityEngine.InputSystem;
using UnityEngine;

namespace AC.Templates.InputSystemIntegration
{

#if UNITY_EDITOR
	[InitializeOnLoad]
#endif
	public class ACCursorPositionProcessor : InputProcessor<Vector2>
	{

		public bool directControlMenuFix = true;

#if UNITY_EDITOR
		static ACCursorPositionProcessor ()
		{
			Initialize ();
		}
#endif

		public override Vector2 Process (Vector2 value, InputControl control)
		{
			if (directControlMenuFix && CanDirectlyControlMenus ())
			{
				return Vector2.zero;
			}
			if (KickStarter.settingsManager.inputMethod == InputMethod.KeyboardOrController)
			{
				return KickStarter.playerInput.GetMousePosition ();
			}
			return value;
		}


		protected bool CanDirectlyControlMenus ()
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

		[RuntimeInitializeOnLoadMethod (RuntimeInitializeLoadType.BeforeSceneLoad)]
		static void Initialize ()
		{
			InputSystem.RegisterProcessor<ACCursorPositionProcessor> ();
		}

	}

}

#endif
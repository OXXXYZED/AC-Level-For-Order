#if UNITY_EDITOR && UNITY_2022_1_OR_NEWER

using System;
using System.Linq;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.UI;
using UnityEditor;
#if InputSystemIsPresent
using UnityEngine.InputSystem;
#endif

namespace AC.Templates.InputSystemIntegration
{

	public class InputSystemTemplate : Template
	{

		#region Variables

#if InputSystemIsPresent
		[SerializeField] private InputActionAsset inputActionsAsset = null;
#endif
		[SerializeField] private MenuManager menuManager = null;

		#endregion


		#region PublicFunctions

		public override bool CanInstall(ref string errorText)
		{
#if InputSystemIsPresent
			if (inputActionsAsset == null)
			{
				errorText = "No Input Actions asset assigned";
				return false;
			}
			if (Resource.References.variablesManager == null)
			{
				errorText = "No Variables Manager assigned";
				return false;
			}
			if (menuManager == null || Resource.References.menuManager == null)
			{
				errorText = "No Menu Manager assigned";
				return false;
			}
			if (menuManager.eventSystem == null)
			{
				errorText = "No Event System assigned";
				return false;
			}
			return true;
#else
			return false;
#endif
		}


		public override bool CanSuggest(NGWData data)
		{
			return true;
		}


		public override bool MeetsDependencyRequirements()
		{
#if InputSystemIsPresent
			return true;
#else
			return false;
#endif
		}

		#endregion


		#region ProtectedFunctions

		protected override void MakeChanges(string installPath, bool canDeleteOldAssets, System.Action onComplete, System.Action<string> onFail)
		{
#if InputSystemIsPresent
			Undo.RecordObjects(new UnityEngine.Object[] { KickStarter.menuManager, KickStarter.variablesManager }, "");

			InputActionAsset newInputActionAsset = CopyAsset<InputActionAsset>(installPath, inputActionsAsset, ".inputactions");
			if (newInputActionAsset == null)
			{
				onFail.Invoke("Input Actions copy failed.");
				return;
			}

			EventSystem newEventSystemPrefab = CopyAsset<EventSystem>(installPath, menuManager.eventSystem, ".prefab");
			if (newEventSystemPrefab == null)
			{
				onFail.Invoke("Prefab copy failed.");
				return;
			}

			var module = newEventSystemPrefab.GetComponent<UnityEngine.InputSystem.UI.InputSystemUIInputModule>();
			var oldModule = menuManager.eventSystem.GetComponent<UnityEngine.InputSystem.UI.InputSystemUIInputModule>();
			module.actionsAsset = newInputActionAsset;

			var playerInput = newEventSystemPrefab.GetComponent<UnityEngine.InputSystem.PlayerInput>();
			UpdatePlayerInputActionsReference(playerInput, newInputActionAsset);

			KickStarter.menuManager.eventSystem = newEventSystemPrefab;

			RemoveExistingMenu("Inputs", !canDeleteOldAssets);
			CopyMenus(installPath, menuManager, KickStarter.menuManager);

			var inputRemapper = KickStarter.menuManager.GetMenuWithName("Inputs").PrefabCanvas.GetComponent<InputRemapper>();
			inputRemapper.SwapInputActionAsset(newInputActionAsset);

			// Variables
			var controlsReader = newEventSystemPrefab.GetComponent<ControlsReader>();
			if (!string.IsNullOrEmpty(controlsReader.GlobalStringVariable))
			{
				GVar dataVar = GetOrCreateGlobalVariable(controlsReader.GlobalStringVariable, VariableType.String);
				dataVar.link = VarLink.OptionsData;
				dataVar.canTranslate = false;
			}

			AssetDatabase.SaveAssets();

			var assets = GetAllAssetReferencesFromAssetDatabase(newInputActionAsset);
			if (assets != null)
			{
				module.point = GetActionReferenceFromAssets(assets, module.point?.action?.name, "Point");
				module.leftClick = GetActionReferenceFromAssets(assets, module.leftClick?.action?.name, "InteractionA");
				module.rightClick = GetActionReferenceFromAssets(assets, module.rightClick?.action?.name, "InteractionB");
				module.scrollWheel = GetActionReferenceFromAssets(assets, module.scrollWheel?.action?.name, "ScrollWheel", "Scroll Wheel", "Scroll", "Wheel");
				module.move = GetActionReferenceFromAssets(assets, module.move?.action?.name, "Navigate");
				module.submit = GetActionReferenceFromAssets(assets, module.submit?.action?.name, "InteractionA");
				module.cancel = GetActionReferenceFromAssets(assets, module.cancel?.action?.name, "InteractionB");
			}
			UnityVersionHandler.CustomSetDirty(module);

			onComplete?.Invoke();
			
#else

			onFail.Invoke("InputSystemIsPresent scripting define symbol not found - is the Input System package installed?");

#endif
		}

		#endregion


		#region PrivateFunctions

#if InputSystemIsPresent

		private InputActionReference[] GetAllAssetReferencesFromAssetDatabase(InputActionAsset actions)
		{
			if (actions == null) return null;

			var path = AssetDatabase.GetAssetPath(actions);
			var assets = AssetDatabase.LoadAllAssetsAtPath(path);
			return assets.Where(asset => asset is InputActionReference)
								.Cast<InputActionReference>()
								.OrderBy(x => x.name)
								.ToArray();
		}


		private InputActionReference GetActionReferenceFromAssets (InputActionReference[] actions, params string[] actionNames)
		{
			foreach (var actionName in actionNames)
			{
				foreach (var action in actions)
				{
					if (action.action != null && string.Compare (action.action.name, actionName, StringComparison.InvariantCultureIgnoreCase) == 0)
					{
						return action;
					}
				}
			}
			return null;
		}



		private void UpdatePlayerInputActionsReference(UnityEngine.InputSystem.PlayerInput playerInput, InputActionAsset newInputActionAsset)
		{
			//playerInput.actions = newInputActionAsset;

			var serializedObject = new UnityEditor.SerializedObject(playerInput);
			var property = serializedObject.FindProperty("m_Actions");

			property.boxedValue = newInputActionAsset;
			serializedObject.ApplyModifiedProperties();

			UnityVersionHandler.CustomSetDirty(playerInput);
		}
		
#endif

		#endregion


		#region GetSet

		public override string Label { get { return "Input System integration"; } }
		public override string PreviewText { get { return "Adds compatibility with Unity's Input System with a custom Event System and Input Actions asset"; } }
		public override Type[] AffectedManagerTypes { get { return new Type[] { typeof(MenuManager), typeof(VariablesManager) }; } }
		public override TemplateCategory Category { get { return TemplateCategory.Interface; } }
		public override int OrderInCategory { get { return 0; } }
		public override bool IsExclusiveToCategory { get { return false; } }
		public override bool RequiresInstallPath { get { return true; } }
		public override string FolderName { get { return "Input System"; } }

		public override bool SelectedByDefault
		{
			get
			{
				var projectSettings = new SerializedObject(UnityEditor.AssetDatabase.LoadAllAssetsAtPath("ProjectSettings/ProjectSettings.asset")[0]);
				if (projectSettings != null)
				{
					var inputHandler = projectSettings.FindProperty("activeInputHandler");
					return inputHandler.intValue == 1;
				}
				return false;
			}
		}

		#endregion

	}


	[CustomEditor (typeof (InputSystemTemplate))]
	public class InputSystemTemplateEditor : TemplateEditor
	{}

}

#endif
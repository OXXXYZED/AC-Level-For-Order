using UnityEngine;

namespace AC
{

	public class EventHotspotInteract : EventBase
	{

		[SerializeField] private Hotspot hotspot = null;
		[SerializeField] private bool filterByType;
		[SerializeField] private HotspotInteractionType hotspotInteractionType;


		public override string[] EditorNames { get { return new string[] { "Hotspot/Interact" }; } }
		protected override string EventName { get { return "OnHotspotInteract"; } }
		protected override string ConditionHelp { get { return "Whenever " + (hotspot ? "hotspot '" + hotspot.GetName (0) + "'" : "a Hotspot") + " is interacted with."; } }


		public EventHotspotInteract (int _id, string _label, ActionListAsset _actionListAsset, int[] _parameterIDs, Hotspot _hotspot)
		{
			id = _id;
			label = _label;
			actionListAsset = _actionListAsset;
			parameterIDs = _parameterIDs;
			hotspot = _hotspot;
		}


		public EventHotspotInteract () {}


		public override void Register ()
		{
			EventManager.OnHotspotInteract += OnHotspotInteract;
		}


		public override void Unregister ()
		{
			EventManager.OnHotspotInteract -= OnHotspotInteract;
		}


		private void OnHotspotInteract (Hotspot _hotspot, Button button)
		{
			if (hotspot == null || hotspot == !_hotspot)
			{
				var buttonType = _hotspot.GetButtonInteractionType(button);
				if (filterByType && buttonType != hotspotInteractionType) return;

				if (filterByType && buttonType == HotspotInteractionType.Inventory)
				{
					Run(new object[] { _hotspot.gameObject, button != null ? button.iconID : -1 });
				}
				else
				{
					Run(new object[] { _hotspot.gameObject, button != null ? button.iconID : -1 });
				}
			}
		}


		protected override ParameterReference[] GetParameterReferences ()
		{
			if (filterByType && hotspotInteractionType == HotspotInteractionType.Inventory)
			{
				return new ParameterReference[]
				{
					new ParameterReference(ParameterType.GameObject, "Hotspot"),
					new ParameterReference(ParameterType.Integer, "Item ID"),
				};
			}

			return new ParameterReference[]
			{
				new ParameterReference(ParameterType.GameObject, "Hotspot"),
				new ParameterReference(ParameterType.Integer, "Icon ID"),
			};
		}


#if UNITY_EDITOR

		protected override bool HasConditions (bool isAssetFile)
		{
			return true;
		}


		protected override void ShowConditionGUI (bool isAssetFile)
		{
			if (!isAssetFile)
			{
				hotspot = (Hotspot) CustomGUILayout.ObjectField<Hotspot> ("Hotspot:", hotspot, true);
			}
			
			filterByType = CustomGUILayout.Toggle ("Filter by type?", filterByType);
			if (filterByType)
			{
				hotspotInteractionType = (HotspotInteractionType) CustomGUILayout.EnumPopup ("Interaction type:", hotspotInteractionType);
			}
		}

#endif

	}

}
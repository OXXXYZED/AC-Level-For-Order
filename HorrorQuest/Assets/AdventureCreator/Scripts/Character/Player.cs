/*
 *
 *	Adventure Creator
 *	by Chris Burton, 2013-2024
 *	
 *	"Player.cs"
 * 
 *	This is attached to the Player GameObject, which must be tagged as Player.
 * 
 */

using System.Collections;
using UnityEngine;
using UnityEngine.Rendering;

namespace AC
{

	/** Attaching this component to a GameObject and tagging it "Player" will make it an Adventure Creator Player. */
	[AddComponentMenu("Adventure Creator/Characters/Player")]
	[HelpURL("https://www.adventurecreator.org/scripting-guide/class_a_c_1_1_player.html")]
	public class Player : NPC
	{

		#region Variables

		public AnimationClip jumpAnim;
		public DetectHotspots hotspotDetector;
		protected int id = -1;
		protected bool lockedPath;
		protected bool lockedPathCanReverse;
		protected float tankTurnFloat;
		public bool toggleRun;

		private Transform directMovementTargetLock;

		public bool LockHotspotHeadTurning { get; set; }
		protected Transform firstPersonCameraTransform;
		protected FirstPersonCamera firstPersonCamera;
		protected bool prepareToJump;
		public bool autoSyncHotspotState = true;

		public float jumpSpeed = 4f;

		protected SkinnedMeshRenderer[] skinnedMeshRenderers;

		public PlayerMoveLock runningLocked = PlayerMoveLock.Free;
		public bool upMovementLocked = false;
		public bool downMovementLocked = false;
		public bool leftMovementLocked = false;
		public bool rightMovementLocked = false;
		public bool freeAimLocked = false;
		public bool jumpingLocked = false;

		#if UNITY_2019_2_OR_NEWER
		public bool autoStickToNavMesh = false;
		private PolygonCollider2D[] autoStickPolys;
		#endif

		// ======= Crouch additions =======

		/** Enable crouch feature */
		[Header("Crouch")]
		public bool crouchEnabled = true;

		/** If true, pressing Crouch toggles crouch on/off. If false, crouch only while the button is held. */
		public bool crouchIsToggle = true;

		/** Movement speed multiplier while crouching (applied to targetSpeed). */
		[Range(0.1f, 1f)] public float crouchSpeedMultiplier = 0.5f;

		/** CharacterController/CapsuleCollider height multiplier while crouching. */
		[Range(0.4f, 0.95f)] public float crouchHeightMultiplier = 0.6f;

		/** Optional Animator boolean parameter name to set while crouching (leave blank to ignore). */
		public string crouchAnimatorBool = "Crouch";

		/** Optional additional local Y-offset for FirstPersonCamera while crouching. */
		public float crouchCameraYOffset = -0.35f;

		private bool isCrouching = false;

		// originals to restore on stand-up
		private float _origControllerHeight;
		private Vector3 _origControllerCenter;

		private float _origCapsuleHeight;
		private Vector3 _origCapsuleCenter;

		private Vector3 _origFpCamLocalPos;

		#endregion


		#region UnityStandards

		protected new void Awake ()
		{
			skinnedMeshRenderers = GetComponentsInChildren <SkinnedMeshRenderer>();

			if (hotspotDetector == null)
			{
				hotspotDetector = GetComponentInChildren<DetectHotspots> ();
			}

			if (KickStarter.playerMovement)
			{
				firstPersonCamera = GetComponentInChildren <FirstPersonCamera>();
				if (firstPersonCamera == null && KickStarter.settingsManager.movementMethod == MovementMethod.FirstPerson && KickStarter.player && KickStarter.player.FirstPersonCamera == null)
				{
					ACDebug.LogWarning ("Could not find a FirstPersonCamera script on the Player - one is necessary for first-person movement.", KickStarter.player);
				}
				if (firstPersonCamera)
				{
					firstPersonCameraTransform = firstPersonCamera.transform;
					_origFpCamLocalPos = firstPersonCameraTransform.localPosition;
				}
			}

			// cache original collider values (if present)
			if (_characterController)
			{
				_origControllerHeight = _characterController.height;
				_origControllerCenter = _characterController.center;
			}
			else
			{
				var capsule = GetComponent<CapsuleCollider>();
				if (capsule)
				{
					_origCapsuleHeight = capsule.height;
					_origCapsuleCenter = capsule.center;
				}
			}

			_Awake ();

			if (GetAnimEngine () != null && KickStarter.settingsManager && KickStarter.settingsManager.hotspotDetection == HotspotDetection.PlayerVicinity && GetAnimEngine ().isSpriteBased && hotspotDetector)
			{
				if (spriteChild && hotspotDetector.transform == spriteChild) {}
				else if (turn2DCharactersIn3DSpace)
				{
					if (hotspotDetector.transform == transform)
					{
						ACDebug.LogWarning ("The Player '" + name + "' has a Hotspot Detector assigned, but it is on the root.  Either parent it to the 'sprite child' instead, or check 'Turn root object in 3D space?' in the Player Inspector.", this);
					}
					else if (hotspotDetector.transform.parent == transform)
					{
						ACDebug.LogWarning ("The Player '" + name + "' has a Hotspot Detector assigned, but it is a direct child of a 2D root.  Either parent it to the 'sprite child' instead, or check 'Turn root object in 3D space?' in the Player Inspector.", this);
					}
				}
			}
		}


		protected override void OnEnable ()
		{
			base.OnEnable ();
			EventManager.OnSetPlayer += OnSetPlayer;
			EventManager.OnBeforeLoading += OnBeforeLoading;
			EventManager.OnInitialiseScene += OnInitialiseScene;
			
			AutoSyncHotspot ();
		}


		protected override void OnDisable ()
		{
			base.OnDisable ();
			EventManager.OnSetPlayer -= OnSetPlayer;
			EventManager.OnBeforeLoading -= OnBeforeLoading;
			EventManager.OnInitialiseScene -= OnInitialiseScene;
		}


		/** The Player's "Update" function, called by StateHandler. */
		public override void _Update ()
		{
			if (!IsActivePlayer ())
			{
				base._Update ();
				return;
			}

			if (firstPersonCamera && !KickStarter.stateHandler.MovementIsOff)
			{
				firstPersonCamera._UpdateFPCamera (false);
			}

			bool jumped = false;
			if (KickStarter.playerInput.InputGetButtonDown ("Jump") && KickStarter.stateHandler.IsInGameplay () && motionControl == MotionControl.Automatic && !KickStarter.stateHandler.MovementIsOff)
			{
				if (!jumpingLocked)
				{
					jumped = Jump ();
				}
			}

			// ======= Crouch input handling =======
			if (crouchEnabled && KickStarter.stateHandler.IsInGameplay () && motionControl == MotionControl.Automatic && !KickStarter.stateHandler.MovementIsOff)
			{
				bool crouchDown = KickStarter.playerInput.InputGetButtonDown("Crouch");
				bool crouchHeld = KickStarter.playerInput.InputGetButton("Crouch");
				bool crouchUp   = KickStarter.playerInput.InputGetButtonUp("Crouch");

				if (crouchIsToggle)
				{
					if (crouchDown)
					{
						SetCrouch (!isCrouching);
					}
				}
				else
				{
					if (crouchDown) SetCrouch (true);
					if (crouchUp)   SetCrouch (false);
					// if control lost (e.g., menu), auto-release
					if (!crouchHeld && isCrouching && KickStarter.stateHandler.gameState != GameState.Normal)
					{
						SetCrouch (false);
					}
				}
			}

			if (hotspotDetector)
			{
				hotspotDetector._Update ();
			}
			
			if (activePath && !pausePath)
			{
				if (IsTurningBeforeWalking ())
				{
					if (charState == CharState.Move)
					{
						StartDecelerating ();
					}
					else if (charState == CharState.Custom)
					{
						charState = CharState.Idle;
					}
				}
				else if ((KickStarter.stateHandler.gameState == GameState.Cutscene && !lockedPath) || 
						(KickStarter.settingsManager.movementMethod == MovementMethod.PointAndClick) ||
						(KickStarter.settingsManager.movementMethod == MovementMethod.None) ||
						(KickStarter.settingsManager.movementMethod == MovementMethod.StraightToCursor && (KickStarter.settingsManager.singleTapStraight || PathfindUpdateFrequency > 0f)) || 
						IsMovingToHotspot ())
				{
					charState = CharState.Move;
				}
			}
			else if (activePath == null && charState == CharState.Move && !KickStarter.stateHandler.IsInGameplay () && KickStarter.stateHandler.gameState != GameState.Paused)
			{
				StartDecelerating ();
			}

			if (isJumping && !jumped)
			{
				if (IsGrounded ())
				{
					isJumping = false;
				}
			}

			BaseUpdate ();
		}


		public override void _LateUpdate ()
		{
			if (firstPersonCamera && !KickStarter.stateHandler.MovementIsOff)
			{
				firstPersonCamera._UpdateFPCamera (true);
			}

			base._LateUpdate ();
			
			#if UNITY_2019_2_OR_NEWER
			if (autoStickToNavMesh && KickStarter.stateHandler.IsInGameplay ())
			{
				SnapToNavMesh2D ();
			}
			#endif
		}


		public override void _FixedUpdate ()
		{
			if (prepareToJump)
			{
				prepareToJump = false;
				_rigidbody.AddForce (UpDirection * jumpSpeed, ForceMode.Impulse);
			}

			base._FixedUpdate ();
		}

		#endregion


		#region PublicFunctions
	
		public void TankTurnLeft (float intensity = 1f)
		{
			Quaternion rot = TransformRotation * Quaternion.Euler (-intensity * turnSpeed * Vector3.up * 60f * Time.deltaTime);
			SetRotation (rot);

			tankTurning = true;
			turnFloat = tankTurnFloat = -intensity;
		}
		
		public void TankTurnRight (float intensity = 1f)
		{
			Quaternion rot = TransformRotation * Quaternion.Euler (intensity * turnSpeed * Vector3.up * 60f * Time.deltaTime);
			SetRotation (rot);

			tankTurning = true;
			turnFloat = tankTurnFloat = intensity;
		}

		public void CancelPathfindRecalculations ()
		{
			pathfindUpdateTime = 0f;
		}

		public override void StopTankTurning ()
		{
			lookDirection = TransformForward;
			tankTurning = false;
		}

		public override float GetTurnFloat ()
		{
			if (tankTurning)
			{
				return tankTurnFloat;
			}
			return base.GetTurnFloat ();
		}

		public void ForceTurnFloat (float _value)
		{
			turnFloat = _value;
		}

		public override bool IsCapableOfJumping ()
		{
			#if UNITY_EDITOR
			if (Application.isPlaying)
			{
				return _characterController || (_rigidbody && !_rigidbody.isKinematic);
			}
			return GetComponent<CharacterController> () || (GetComponent<Rigidbody> () && !GetComponent<Rigidbody>().isKinematic);
			#else
			return _characterController || (_rigidbody && !_rigidbody.isKinematic);
			#endif
		}

		/**
		 * Causes the Player to jump, so long as a Rigidbody or Character Controller component is attached.
		 */
		public bool Jump (bool mustBeGrounded = true, bool mustNotBeMidJump = true)
		{
			if (isJumping && mustNotBeMidJump)
			{
				return false;
			}

			bool isGrounded = IsGrounded ();

			if (activePath == null)
			{
				if (_characterController)
				{
					if (!isGrounded && mustBeGrounded)
					{
						RaycastHit hitDownInfo;
						bool hitGround = Physics.Raycast (Transform.position + UpDirection * _characterController.stepOffset, -UpDirection, out hitDownInfo, _characterController.stepOffset * 2f, groundCheckLayerMask);
						if (!hitGround)
						{
							return false;
						}
					}

					simulatedVerticalSpeed = jumpSpeed * 0.1f;
					isJumping = true;
					_characterController.Move (simulatedVerticalSpeed * Time.deltaTime * UpDirection);
					KickStarter.eventManager.Call_OnPlayerJump (this);
					return true;
				}
				else if (_rigidbody && !_rigidbody.isKinematic && (isGrounded || !mustBeGrounded))
				{
					if (useRigidbodyForMovement)
					{	
						prepareToJump = true;
					}
					else
					{
						UnityVersionHandler.SetRigidbodyVelocity (_rigidbody, UpDirection * jumpSpeed);
					}
					isJumping = true;

					if (ignoreGravity)
					{
						ACDebug.LogWarning (gameObject.name + " is jumping - but 'Ignore gravity?' is enabled in the Player Inspector. Is this correct?", gameObject);
					}
					KickStarter.eventManager.Call_OnPlayerJump (this);
					return true;
				}
				else if (isGrounded || !mustBeGrounded)
				{
					if (motionControl == MotionControl.Automatic)
					{
						if (_rigidbody && _rigidbody.isKinematic)
						{
							ACDebug.Log ("Player cannot jump without a non-kinematic Rigidbody component.", gameObject);
						}
						else
						{
							ACDebug.Log ("Player cannot jump without a Rigidbody or Character Controller.", gameObject);
						}
						KickStarter.eventManager.Call_OnPlayerJump (this);
					}
				}
			}
			else if (_collider == null)
			{
				ACDebug.Log (gameObject.name + " has no Collider component", gameObject);
			}

			return false;
		}

		public override void EndPath ()
		{
			if (lockedPath)
			{
				if (activePath) activePath.pathType = lockedPathType;
				lockedPath = false;
			}
			base.EndPath ();
		}

		public void ReverseDirectPathDirection ()
		{
			if (lockedPath && lockedPathCanReverse)
			{
				switch (activePath.pathType)
				{
					case AC_PathType.ForwardOnly:
					case AC_PathType.Loop:
						activePath.pathType = AC_PathType.ReverseOnly;
						targetNode --;
						if (targetNode < 0) targetNode = activePath.nodes.Count - 1;
						PathUpdate ();
						break;

					case AC_PathType.ReverseOnly:
						activePath.pathType = AC_PathType.ForwardOnly;
						targetNode ++;
						if (targetNode >= activePath.nodes.Count) targetNode = 0;
						PathUpdate ();
						break;

					default:
						break;
				}
			}
		}

		public void SetLockedPath (Paths pathOb, bool canReverse = false, PathSnapping pathSnapping = PathSnapping.SnapToStart, int startingNode = 0)
		{
			if (KickStarter.settingsManager.movementMethod == MovementMethod.Direct || KickStarter.settingsManager.movementMethod == MovementMethod.FirstPerson)
			{
				lockedPath = true;
				lockedPathType = pathOb.pathType;

				if (KickStarter.settingsManager.movementMethod == MovementMethod.Direct)
				{
					lockedPathCanReverse = canReverse;
				}
				else
				{
					lockedPathCanReverse = false;
				}

				if (pathOb.pathSpeed == PathSpeed.Run)
				{
					isRunning = true;
				}
				else
				{
					isRunning = false;
				}

				switch (pathSnapping)
				{
					default:
						startingNode = pathOb.GetNearestNode (Transform.position);
						break;

					case PathSnapping.SnapToStart:
						startingNode = 0;
						break;

					case PathSnapping.SnapToNode:
						break;
				}

				if (pathOb.nodes == null || pathOb.nodes.Count == 0 || startingNode >= pathOb.nodes.Count)
				{
					lockedPath = false;
					ACDebug.LogWarning ("Cannot lock Player to path '" + pathOb + "' - invalid node index " + startingNode, pathOb);
					return;
				}

				Vector3 pathPosition = pathOb.nodes[startingNode];

				if (pathSnapping != PathSnapping.None)
				{
					if (pathOb.affectY)
					{
						Teleport (pathPosition);
					}
					else if (SceneSettings.IsUnity2D ())
					{
						Teleport (new Vector3 (pathPosition.x, pathPosition.y, Transform.position.z));
					}
					else
					{
						Teleport (new Vector3 (pathPosition.x, Transform.position.y, pathPosition.z));
					}
				}
					
				activePath = pathOb;

				if (startingNode == pathOb.nodes.Count - 1 && lockedPathType == AC_PathType.Loop)
				{
					targetNode = 0;
				}
				else
				{
					targetNode = startingNode + 1;
				}

				if (startingNode == pathOb.nodes.Count - 1)
				{
					if (lockedPathCanReverse)
					{
						activePath.pathType = AC_PathType.ReverseOnly;
						targetNode = startingNode - 1;
					}
					else
					{
						ACDebug.LogWarning ("Cannot lock Player to path '" + pathOb + "' - node index " + startingNode + " is the end of the path, and bi-directional movement is disabled.", pathOb);

						Vector3 direction = Transform.position - pathOb.nodes[targetNode-2];
						Vector3 lookDir = Flatten3D (direction);
						SetLookDirection (lookDir, true);

						lockedPath = false;
						activePath = null;
					}
				}

				if (activePath)
				{
					Vector3 direction = activePath.nodes[targetNode] - Transform.position;
					Vector3 lookDir = Flatten3D (direction);
					SetLookDirection (lookDir, true);
				}

				charState = CharState.Idle;
			}
			else
			{
				ACDebug.LogWarning ("Path-constrained player movement is only available with Direct control.", gameObject);
			}
		}

		public bool IsLockedToPath () { return lockedPath; }

		public bool AllDirectionsLocked ()
		{
			if (downMovementLocked && upMovementLocked && leftMovementLocked && rightMovementLocked) return true;
			return false;
		}

		public bool IsTilting ()
		{
			if (firstPersonCamera)
			{
				return firstPersonCamera.IsTilting ();
			}
			return false;
		}

		public float GetTilt ()
		{
			if (firstPersonCamera)
			{
				return firstPersonCamera.GetTilt ();;
			}
			return 0f;
		}
		
		public void SetTilt (Vector3 lookAtPosition, bool isInstant)
		{
			if (firstPersonCameraTransform == null || firstPersonCamera == null) return;
			
			if (isInstant)
			{
				Vector3 lookDirection = (lookAtPosition - firstPersonCameraTransform.position).normalized;
				float angle = Mathf.Asin (lookDirection.y) * Mathf.Rad2Deg;
				firstPersonCamera.SetPitch (-angle);
			}
			else
			{
				Quaternion oldRotation = firstPersonCameraTransform.rotation;
				firstPersonCameraTransform.LookAt (lookAtPosition);
				float targetTilt = firstPersonCameraTransform.localEulerAngles.x;
				firstPersonCameraTransform.rotation = oldRotation;
				if (targetTilt > 180) targetTilt = targetTilt - 360;
				firstPersonCamera.SetPitch (targetTilt, false);
			}
		}

		public void SetTilt (float pitchAngle, bool isInstant)
		{
			if (firstPersonCamera == null) return;
			firstPersonCamera.SetPitch (pitchAngle, isInstant);
		}

		public override void SetHeadTurnTarget (Transform _headTurnTarget, Vector3 _headTurnTargetOffset, bool isInstant, HeadFacing _headFacing = HeadFacing.Manual)
		{
			if (!IsActivePlayer ())
			{
				base.SetHeadTurnTarget (_headTurnTarget, _headTurnTargetOffset, isInstant, _headFacing);
				return;
			}

			if (_headFacing == HeadFacing.Hotspot && LockHotspotHeadTurning)
			{
				ClearHeadTurnTarget (false, HeadFacing.Hotspot);
			}
			else
			{
				base.SetHeadTurnTarget (_headTurnTarget, _headTurnTargetOffset, isInstant, _headFacing);
			}
		}

		public PlayerData SaveData (PlayerData playerData)
		{
			playerData.playerID = ID;
			
			playerData.playerLocX = Transform.position.x;
			playerData.playerLocY = Transform.position.y;
			playerData.playerLocZ = Transform.position.z;
			playerData.playerRotY = TransformRotation.eulerAngles.y;

			playerData.inCustomCharState = (charState == CharState.Custom && GetAnimator () && GetAnimator ().GetComponent <RememberAnimator>());
			
			playerData.playerWalkSpeed = walkSpeedScale;
			playerData.playerRunSpeed = runSpeedScale;

			playerData.playerUpLock = upMovementLocked;
			playerData.playerDownLock = downMovementLocked;
			playerData.playerLeftlock = leftMovementLocked;
			playerData.playerRightLock = rightMovementLocked;
			playerData.playerRunLock = (int) runningLocked;
			playerData.playerFreeAimLock = freeAimLocked;

			if (GetAnimEngine () != null)
			{
				playerData = GetAnimEngine ().SavePlayerData (playerData, this);
			}
						
			playerData.playerPortraitGraphic = AssetLoader.GetAssetInstanceID (portraitIcon.texture);
			playerData.playerSpeechLabel = GetName ();
			playerData.playerDisplayLineID = displayLineID;

			playerData.playerLockDirection = lockDirection;
			playerData.playerLockScale = lockScale;
			if (spriteChild && spriteChild.GetComponent <FollowSortingMap>())
			{
				playerData.playerLockSorting = spriteChild.GetComponent <FollowSortingMap>().lockSorting;
			}
			else if (GetComponent <FollowSortingMap>())
			{
				playerData.playerLockSorting = GetComponent <FollowSortingMap>().lockSorting;
			}
			else
			{
				playerData.playerLockSorting = false;
			}

			playerData.playerSpriteDirection = GetSpriteDirectionToSave ();

			playerData.playerSpriteScale = spriteScale;
			var sortingGroup = GetComponentInChildren<SortingGroup> ();
			if (sortingGroup)
			{
				playerData.playerSortingOrder = sortingGroup.sortingOrder;
				playerData.playerSortingLayer = sortingGroup.sortingLayerName;
			}
			else if (spriteChild && spriteChild.GetComponent <Renderer>())
			{
				playerData.playerSortingOrder = spriteChild.GetComponent <Renderer>().sortingOrder;
				playerData.playerSortingLayer = spriteChild.GetComponent <Renderer>().sortingLayerName;
			}
			else if (GetComponent <Renderer>())
			{
				playerData.playerSortingOrder = GetComponent <Renderer>().sortingOrder;
				playerData.playerSortingLayer = GetComponent <Renderer>().sortingLayerName;
			}
			
			playerData.playerActivePath = 0;
			playerData.lastPlayerActivePath = 0;
			playerData.playerPathData = string.Empty;

			if (GetPath ())
			{
				playerData.playerTargetNode = GetTargetNode ();
				playerData.playerPrevNode = GetPreviousNode ();
				playerData.playerIsRunning = isRunning;
				playerData.playerPathAffectY = activePath.affectY;

				if (GetPath () == ownPath)
				{
					playerData.playerPathData = Serializer.CreatePathData (ownPath);
					playerData.playerLockedPath = false;
				}
				else
				{
					playerData.playerPathData = string.Empty;
					playerData.playerActivePath = Serializer.GetConstantID (GetPath ().gameObject);
					playerData.playerLockedPath = lockedPath;
					playerData.playerLockedPathReversing = lockedPathCanReverse;
					playerData.playerLockedPathType = (int) lockedPathType;
				}
			}
			
			if (GetLastPath ())
			{
				playerData.lastPlayerTargetNode = GetLastTargetNode ();
				playerData.lastPlayerPrevNode = GetLastPrevNode ();
				playerData.lastPlayerActivePath = Serializer.GetConstantID (GetLastPath ().gameObject);
			}
			
			playerData.playerIgnoreGravity = ignoreGravity;
			
			playerData.playerLockHotspotHeadTurning = LockHotspotHeadTurning;
			if (headFacing == HeadFacing.Manual && headTurnTarget)
			{
				playerData.isHeadTurning = true;
				playerData.headTargetID = Serializer.GetConstantID (headTurnTarget);
				if (playerData.headTargetID == 0)
				{
					ACDebug.LogWarning ("The Player's head-turning target Transform, " + headTurnTarget + ", was not saved because it has no Constant ID", gameObject);
				}
				playerData.headTargetX = headTurnTargetOffset.x;
				playerData.headTargetY = headTurnTargetOffset.y;
				playerData.headTargetZ = headTurnTargetOffset.z;
			}
			else
			{
				playerData.isHeadTurning = false;
				playerData.headTargetID = 0;
				playerData.headTargetX = 0f;
				playerData.headTargetY = 0f;
				playerData.headTargetZ = 0f;
			}

			playerData.fpCameraPitch = firstPersonCamera ? firstPersonCamera.GetTargetTilt () : 0f;

			FollowSortingMap followSortingMap = GetComponentInChildren <FollowSortingMap>();
			if (followSortingMap)
			{
				playerData.followSortingMap = followSortingMap.followSortingMap;
				if (!playerData.followSortingMap && followSortingMap.GetSortingMap ())
				{
					ConstantID followSortingMapConstantID = followSortingMap.GetSortingMap ().GetComponent <ConstantID>();

					if (followSortingMapConstantID)
					{
						playerData.customSortingMapID = followSortingMapConstantID.constantID;
					}
					else
					{
						ACDebug.LogWarning ("The Player's SortingMap, " + followSortingMap.GetSortingMap ().name + ", was not saved because it has no Constant ID", gameObject);
						playerData.customSortingMapID = 0;
					}
				}
				else
				{
					playerData.customSortingMapID = 0;
				}
			}
			else
			{
				playerData.followSortingMap = false;
				playerData.customSortingMapID = 0;
			}

			if (followTarget && !IsActivePlayer ())
			{
				if (!followTargetIsPlayer)
				{
					if (followTarget.GetComponent<ConstantID> ())
					{
						playerData.followTargetID = followTarget.GetComponent<ConstantID> ().constantID;
						playerData.followTargetIsPlayer = followTargetIsPlayer;
						playerData.followFrequency = followFrequency;
						playerData.followDistance = followDistance;
						playerData.followDistanceMax = followDistanceMax;
						playerData.followFaceWhenIdle = followFaceWhenIdle;
						playerData.followRandomDirection = followRandomDirection;
						playerData.followAcrossScenes = false;
					}
					else
					{
						ACDebug.LogWarning ("Want to save follow data for " + name + " but " + followTarget.name + " has no ID!", gameObject);
					}
				}
				else
				{
					playerData.followTargetID = 0;
					playerData.followTargetIsPlayer = followTargetIsPlayer;
					playerData.followFrequency = followFrequency;
					playerData.followDistance = followDistance;
					playerData.followDistanceMax = followDistanceMax;
					playerData.followFaceWhenIdle = followFaceWhenIdle;
					playerData.followRandomDirection = followRandomDirection;
					playerData.followAcrossScenes = followAcrossScenes;
				}
			}
			else
			{
				playerData.followTargetID = 0;
				playerData.followTargetIsPlayer = false;
				playerData.followFrequency = 0f;
				playerData.followDistance = 0f;
				playerData.followDistanceMax = 0f;
				playerData.followFaceWhenIdle = false;
				playerData.followRandomDirection = false;
				playerData.followAcrossScenes = false;
			}

			playerData.leftHandIKState = LeftHandIKController.CreateSaveData ();
			playerData.rightHandIKState = RightHandIKController.CreateSaveData ();

			playerData.attachmentPointDatas = new AttachmentPointData[attachmentPoints.Length];
			for (int i = 0; i < attachmentPoints.Length; i++)
			{
				playerData.attachmentPointDatas[i] = new AttachmentPointData (attachmentPoints[i]);
			}

			playerData.spriteDirectionData = spriteDirectionData.SaveData ();

			if (!IsLocalPlayer () && gameObject.activeInHierarchy)
			{
				playerData = KickStarter.levelStorage.SavePlayerData (this, playerData);
			}
			
			return playerData;
		}

		public bool IsLocalPlayer () { return (ID <= -2); }

		public IEnumerator LoadData (PlayerData playerData)
		{
			upMovementLocked = playerData.playerUpLock;
			downMovementLocked = playerData.playerDownLock;
			leftMovementLocked = playerData.playerLeftlock;
			rightMovementLocked = playerData.playerRightLock;
			runningLocked = (PlayerMoveLock) playerData.playerRunLock;
			freeAimLocked = playerData.playerFreeAimLock;

			charState = (playerData.inCustomCharState) ? CharState.Custom : CharState.Idle;

			playerData.UpdateFromTempPosition (this);
				
			Teleport (new Vector3 (playerData.playerLocX, playerData.playerLocY, playerData.playerLocZ));
			SetRotation (playerData.playerRotY);
			SetMoveDirectionAsForward ();

			walkSpeedScale = playerData.playerWalkSpeed;
			runSpeedScale = playerData.playerRunSpeed;
			
			GetAnimEngine ().LoadPlayerData (playerData, this);

			SetName (playerData.playerSpeechLabel, playerData.playerDisplayLineID);
			
			lockDirection = playerData.playerLockDirection;
			lockScale = playerData.playerLockScale;
			if (spriteChild && spriteChild.GetComponent <FollowSortingMap>())
			{
				spriteChild.GetComponent <FollowSortingMap>().lockSorting = playerData.playerLockSorting;
			}
			else if (GetComponent <FollowSortingMap>())
			{
				GetComponent <FollowSortingMap>().lockSorting = playerData.playerLockSorting;
			}
			else
			{
				ReleaseSorting ();
			}
			
			if (playerData.playerLockDirection)
			{
				spriteDirection = playerData.playerSpriteDirection;
				UpdateFrameFlipping (true);
			}
			if (playerData.playerLockScale)
			{
				spriteScale = playerData.playerSpriteScale;
			}
			if (playerData.playerLockSorting)
			{
				var sortingGroup = GetComponentInChildren<SortingGroup> ();
				if (sortingGroup)
				{
					sortingGroup.sortingOrder = playerData.playerSortingOrder;
					sortingGroup.sortingLayerName = playerData.playerSortingLayer;
				}
				else if (spriteChild && spriteChild.GetComponent <Renderer>())
				{
					spriteChild.GetComponent <Renderer>().sortingOrder = playerData.playerSortingOrder;
					spriteChild.GetComponent <Renderer>().sortingLayerName = playerData.playerSortingLayer;
				}
				else if (GetComponent <Renderer>())
				{
					GetComponent <Renderer>().sortingOrder = playerData.playerSortingOrder;
					GetComponent <Renderer>().sortingLayerName = playerData.playerSortingLayer;
				}
			}

			AC.Char charToFollow = null;
			if (playerData.followTargetID != 0)
			{
				RememberNPC followNPC = ConstantID.GetComponent <RememberNPC> (playerData.followTargetID);
				if (followNPC.GetComponent<AC.Char> ())
				{
					charToFollow = followNPC.GetComponent<AC.Char> ();
				}
			}

			if (charToFollow != null || (playerData.followTargetIsPlayer && KickStarter.player))
			{
				FollowAssign (charToFollow, playerData.followTargetIsPlayer, playerData.followFrequency, playerData.followDistance, playerData.followDistanceMax, playerData.followFaceWhenIdle, playerData.followRandomDirection, playerData.followAcrossScenes);
			}
			else
			{
				StopFollowing ();
			}

			Halt ();

			if (!playerData.inCustomCharState)
			{
				ForceIdle ();
			}

			if (!string.IsNullOrEmpty (playerData.playerPathData) && ownPath)
			{
				Paths savedPath = ownPath;
				savedPath = Serializer.RestorePathData (savedPath, playerData.playerPathData);
				SetPath (savedPath, playerData.playerTargetNode, playerData.playerPrevNode, playerData.playerPathAffectY);
				isRunning = playerData.playerIsRunning;
				lockedPath = false;
			}
			else if (playerData.playerActivePath != 0)
			{
				Paths savedPath = ConstantID.GetComponent <Paths> (playerData.playerActivePath);
				if (savedPath)
				{
					lockedPath = playerData.playerLockedPath;
					
					if (lockedPath)
					{
						savedPath.pathType = AC_PathType.ForwardOnly;
						SetLockedPath (savedPath, playerData.playerLockedPathReversing);
						lockedPathType = (AC_PathType) playerData.playerLockedPathType;
						
						Teleport (new Vector3 (playerData.playerLocX, playerData.playerLocY, playerData.playerLocZ));
						SetRotation (playerData.playerRotY);
						targetNode = playerData.playerTargetNode;
						prevNode = playerData.playerPrevNode;
					}
					else
					{
						SetPath (savedPath, playerData.playerTargetNode, playerData.playerPrevNode);
					}
				}
				else
				{
					Halt ();
					ForceIdle ();
				}
			}
			else
			{
				Halt ();
				if (!playerData.inCustomCharState)
				{
					ForceIdle ();
				}
			}
			
			if (playerData.lastPlayerActivePath != 0)
			{
				Paths savedPath = ConstantID.GetComponent <Paths> (playerData.lastPlayerActivePath);
				if (savedPath)
				{
					SetLastPath (savedPath, playerData.lastPlayerTargetNode, playerData.lastPlayerPrevNode);
				}
			}
			
			if (firstPersonCamera)
			{
				firstPersonCamera.SetPitch (playerData.fpCameraPitch);
			}

			LockHotspotHeadTurning = playerData.playerLockHotspotHeadTurning;
			if (playerData.isHeadTurning)
			{
				ConstantID _headTargetID = ConstantID.GetComponent <ConstantID> (playerData.headTargetID);
				if (_headTargetID)
				{
					SetHeadTurnTarget (_headTargetID.transform, new Vector3 (playerData.headTargetX, playerData.headTargetY, playerData.headTargetZ), true);
				}
				else
				{
					ClearHeadTurnTarget (true);
				}
			}
			else
			{
				ClearHeadTurnTarget (true);
			}
			
			ignoreGravity = playerData.playerIgnoreGravity;

			FollowSortingMap[] followSortingMaps = GetComponentsInChildren <FollowSortingMap>();
			if (followSortingMaps != null && followSortingMaps.Length > 0)
			{
				SortingMap customSortingMap = ConstantID.GetComponent <SortingMap> (playerData.customSortingMapID);
				
				foreach (FollowSortingMap followSortingMap in followSortingMaps)
				{
					followSortingMap.followSortingMap = playerData.followSortingMap;
					if (!playerData.followSortingMap && customSortingMap)
					{
						followSortingMap.SetSortingMap (customSortingMap);
					}
					else
					{
						followSortingMap.SetSortingMap (KickStarter.sceneSettings.sortingMap);
					}
				}
			}

			ignoreGravity = playerData.playerIgnoreGravity;

			if (GetAnimEngine () != null && GetAnimEngine ().IKEnabled)
			{
				LeftHandIKController.LoadData (playerData.leftHandIKState);
				RightHandIKController.LoadData (playerData.rightHandIKState);
			}

			_spriteDirectionData.LoadData (playerData.spriteDirectionData);

			if ((playerData.leftHandSceneItemConstantID != 0 || playerData.rightHandSceneItemConstantID != 0) && playerData.attachmentPointDatas.Length == 0)
			{
				playerData.attachmentPointDatas = new AttachmentPointData[2] { new AttachmentPointData (0, playerData.leftHandSceneItemConstantID), new AttachmentPointData (1, playerData.rightHandSceneItemConstantID) };
			}

			GameObject[] heldObjectsToSpawn = (playerData.attachmentPointDatas != null) ? new GameObject[playerData.attachmentPointDatas.Length] : new GameObject[0];
			for (int i = 0; i < heldObjectsToSpawn.Length; i++)
			{
				if (playerData.attachmentPointDatas[i].heldSceneItemConstantID != 0 && i < attachmentPoints.Length && attachmentPoints[i].transform)
				{
					foreach (ScriptData scriptData in playerData.playerScriptData)
					{
						if (scriptData.objectID == playerData.attachmentPointDatas[i].heldSceneItemConstantID)
						{
							SceneItemData data = Serializer.LoadScriptData<SceneItemData> (scriptData.data);
							if (data == null) continue;

							var attachmentPoint = GetAttachmentPoint (playerData.attachmentPointDatas[i].attachmentPointID);
							if (attachmentPoint == null) continue;

							InvInstance invInstance = InvInstance.LoadData (data.invInstanceData);
							if (!InvInstance.IsValid (invInstance) || invInstance.InvItem.linkedPrefab == null) continue;

							heldObjectsToSpawn[i] = Instantiate (invInstance.InvItem.linkedPrefab);
							heldObjectsToSpawn[i].name = invInstance.InvItem.linkedPrefab.name;
							heldObjectsToSpawn[i].GetComponent<RememberSceneItem> ().SetManualID (playerData.attachmentPointDatas[i].heldSceneItemConstantID);
							heldObjectsToSpawn[i].transform.SetParent (attachmentPoint.transform);
							heldObjectsToSpawn[i].transform.localPosition = Vector3.zero;
							heldObjectsToSpawn[i].transform.localEulerAngles = Vector3.zero;
						}
					}
				}
			}

			if (IsLocalPlayer ())
			{
				if (GetAnimator ()) GetAnimator ().Update (0f);
			}
			else
			{
				var loadPlayerScriptDataCoroutine = KickStarter.levelStorage.LoadPlayerData (this, playerData);
				while (loadPlayerScriptDataCoroutine.MoveNext ())
				{
					yield return loadPlayerScriptDataCoroutine.Current;
				}
			}

			for (int i = 0; i < heldObjectsToSpawn.Length; i++)
			{
				if (i < attachmentPoints.Length)
					HoldObject (heldObjectsToSpawn[i], attachmentPoints[i].ID);
			}
		}

		private AttachmentPoint GetAttachmentPoint (int ID)
		{
			if (attachmentPoints == null) return null;
			foreach (var attachmentPoint in attachmentPoints)
			{
				if (attachmentPoint.ID == ID) return attachmentPoint;
			}
			return null;
		}

		public virtual void Hide ()
		{
			foreach (SkinnedMeshRenderer skinnedMeshRenderer in skinnedMeshRenderers)
			{
				skinnedMeshRenderer.enabled = false;
			}
		}

		public virtual void Show ()
		{
			foreach (SkinnedMeshRenderer skinnedMeshRenderer in skinnedMeshRenderers)
			{
				skinnedMeshRenderers[0].enabled = true;
			}
			foreach (SkinnedMeshRenderer skinnedMeshRenderer in skinnedMeshRenderers)
			{
				skinnedMeshRenderer.enabled = true;
			}
		}

		public void RemoveFromScene (bool immediately = false)
		{
			if (KickStarter.eventManager)
			{
				KickStarter.eventManager.Call_OnPlayerRemove (this);
			}

			KickStarter.dialog.EndSpeechByCharacter (this);
			
			Renderer[] playerObRenderers = gameObject.GetComponentsInChildren<Renderer> ();
			foreach (Renderer renderer in playerObRenderers)
			{
				renderer.enabled = false;
			}

			Collider[] playerObColliders = gameObject.GetComponentsInChildren<Collider> ();
			foreach (Collider collider in playerObColliders)
			{
				if (collider is CharacterController) continue;
				collider.isTrigger = true;
			}

			if (!IsLocalPlayer () && id >= 0)
			{
				PlayerPrefab playerPrefab = KickStarter.settingsManager.GetPlayerPrefab (id);
				if (playerPrefab != null)
				{
					KickStarter.playerSpawner.ReleaseHandle (playerPrefab);
				}
			}

			if (immediately)
			{
				DestroyImmediate (gameObject);
			}
			else
			{
				KickStarter.sceneChanger.ScheduleForDeletion (gameObject);
			}
		}

		public bool IsFollowingActivePlayerAcrossScenes ()
		{
			if (followAcrossScenes && followTargetIsPlayer)
			{ 
				return true;
			}
			return false;
		}

		public override string ToString ()
		{
			string prefix = IsActivePlayer () ? "Active Player " : "Player ";
			if (!string.IsNullOrEmpty (speechLabel))
			{
				return prefix + speechLabel;
			}
			return prefix + name;
		}

		// ======= Crouch public API =======

		/** Returns true if player is currently crouching. */
		public bool IsCrouching () => isCrouching;

		/** Force-set crouch state (performs safety ceiling check when standing up). */
		public void SetCrouch (bool value)
		{
			if (!crouchEnabled) return;
			if (value == isCrouching) return;

			if (!value)
			{
				// want to stand up: ensure we have headroom
				if (!HasHeadroomToStand ())
				{
					return;
				}
			}

			isCrouching = value;
			ApplyCrouchToColliders (isCrouching);
			ApplyCrouchToCamera (isCrouching);
			ApplyCrouchToAnimator (isCrouching);
		}

		#endregion


		#region ProtectedFunctions

		protected void SnapToNavMesh2D ()
		{
			if (IsMovingAlongPath () || !SceneSettings.IsUnity2D () || KickStarter.sceneSettings == null || KickStarter.sceneSettings.navMesh == null || KickStarter.settingsManager.movementMethod == MovementMethod.PointAndClick) return;

			#if UNITY_2019_2_OR_NEWER

			if (autoStickPolys == null || autoStickPolys.Length == 0 || autoStickPolys[0] == null || autoStickPolys[0].gameObject != KickStarter.sceneSettings.navMesh.gameObject)
			{
				autoStickPolys = KickStarter.sceneSettings.navMesh.GetComponents<PolygonCollider2D> ();
			}

			float minSqrDist = 0f;
			int bestIndex = -1;
			Vector3 bestPosition = Vector3.zero;

			for (int i = 0; i < autoStickPolys.Length; i++)
			{
				if (autoStickPolys[i] == null) continue;

				Vector3 newPosition = autoStickPolys[i].ClosestPoint (transform.position);
				
				float sqrDist = (newPosition - transform.position).sqrMagnitude;
				if (sqrDist == 0f)
				{
					return;
				}

				if (bestIndex < 0 || sqrDist < minSqrDist)
				{
					minSqrDist = sqrDist;
					bestIndex = i;
					bestPosition = newPosition;
				}
			}

			if (bestIndex >= 0)
			{
				Teleport (bestPosition);
			}
			#endif
		}

		protected override bool CanBeDirectControlled ()
		{
			if (KickStarter.stateHandler.gameState == GameState.Normal)
			{
				if (KickStarter.settingsManager.movementMethod == MovementMethod.Direct || KickStarter.settingsManager.movementMethod == MovementMethod.FirstPerson)
				{
					return !upMovementLocked && !downMovementLocked && !leftMovementLocked && !rightMovementLocked;
				}
			}
			return false;
		}
		
		protected bool IsMovingToHotspot ()
		{
			if (KickStarter.playerInteraction && KickStarter.playerInteraction.GetHotspotMovingTo ())
			{
				return true;
			}
			
			return false;
		}

		protected void OnSetPlayer (Player player)
		{
			AutoSyncHotspot ();
		}

		protected void OnBeforeLoading (SaveFile saveFile)
		{
			for (int i = 0; i < attachmentPoints.Length; i++)
			{
				if (attachmentPoints[i].heldObject)
				{
					RememberSceneItem rememberSceneItem = attachmentPoints[i].heldObject.GetComponent<RememberSceneItem> ();
					if (rememberSceneItem)
					{
						DestroyImmediate (attachmentPoints[i].heldObject);
					}
				}
			}
		}

		protected void OnInitialiseScene ()
		{
			autoStickPolys = null;
		}

		protected void AutoSyncHotspot ()
		{
			bool enable = (KickStarter.player == null || KickStarter.player != this);

			if (autoSyncHotspotState && KickStarter.settingsManager.playerSwitching == PlayerSwitching.Allow)
			{
				Hotspot[] hotspots = GetComponentsInChildren<Hotspot> ();
				foreach (Hotspot hotspot in hotspots)
				{
					if (enable) hotspot.TurnOn ();
					else hotspot.TurnOff ();
				}
			}
		}

		protected override void Accelerate ()
		{
			if (!IsActivePlayer ())
			{
				base.Accelerate ();
				return;
			}

			float targetSpeed = GetTargetSpeed ();

			// slow down while crouched
			if (isCrouching)
			{
				targetSpeed *= Mathf.Clamp(crouchSpeedMultiplier, 0.1f, 1f);
			}

			if (AccurateDestination () && WillStopAtNextNode ())
			{
				AccurateAcc (GetTargetSpeed (), false);
			}
			else
			{
				if (KickStarter.settingsManager.magnitudeAffectsDirect && KickStarter.settingsManager.movementMethod == MovementMethod.Direct && KickStarter.stateHandler.IsInGameplay () && !IsMovingToHotspot ())
				{
					targetSpeed -= (1f - KickStarter.playerInput.GetMoveKeys ().magnitude) / 2f;
				}

				moveSpeed = moveSpeedLerp.Update (moveSpeed, targetSpeed, acceleration);
			}
		}

		#endregion


		#region GetSet

		public Transform FirstPersonCamera
		{
			get
			{
				if (firstPersonCameraTransform == null && FirstPersonCameraComponent)
				{
					firstPersonCameraTransform = FirstPersonCamera.transform;
				}
				return firstPersonCameraTransform;
			}
			set
			{
				firstPersonCameraTransform = value;
			}
		}

		public FirstPersonCamera FirstPersonCameraComponent
		{
			get
			{
				if (firstPersonCamera == null)
				{
					firstPersonCamera = GetComponentInChildren <FirstPersonCamera>();
				}
				return firstPersonCamera;
			}
		}

		public InvCollection Inventory
		{
			get
			{
				if (KickStarter.settingsManager && KickStarter.settingsManager.playerSwitching == PlayerSwitching.Allow && KickStarter.player != this)
				{
					return KickStarter.saveSystem.GetItemsFromPlayer (ID);
				}
				return KickStarter.runtimeInventory.PlayerInvCollection;
			}
		}

		public override bool IsPlayer { get { return true; } }

		public override bool IsActivePlayer () { return this == KickStarter.player; }

		public Transform DirectMovementTargetLock
		{
			get { return directMovementTargetLock; }
			set { directMovementTargetLock = value; }
		}

		public int ID
		{
			get { return id; }
			set { StartCoroutine (SetID (value)); }
		}

		public IEnumerator SetID (int value)
		{
			id = value;
			if (id < -1 && KickStarter.settingsManager && KickStarter.settingsManager.playerSwitching == PlayerSwitching.Allow)
			{
				ACDebug.LogWarning ("The use of 'in-scene' local Players is not recommended when Player-switching is enabled - consider using the 'Player: Switch' Action to change Player instead.");
			}

			var assignPlayerDataCoroutine = KickStarter.saveSystem.AssignPlayerData (this);
			while (assignPlayerDataCoroutine.MoveNext ())
			{
				yield return assignPlayerDataCoroutine.Current;
			}
		}

		#endregion


		#region Crouch internals

		private void ApplyCrouchToAnimator (bool crouched)
		{
			if (string.IsNullOrEmpty(crouchAnimatorBool)) return;
			var anim = GetAnimator();
			if (anim) anim.SetBool(crouchAnimatorBool, crouched);
		}

		private void ApplyCrouchToCamera (bool crouched)
		{
			if (!firstPersonCameraTransform) return;

			if (crouched)
			{
				firstPersonCameraTransform.localPosition = _origFpCamLocalPos + new Vector3(0f, crouchCameraYOffset, 0f);
			}
			else
			{
				firstPersonCameraTransform.localPosition = _origFpCamLocalPos;
			}
		}

		private void ApplyCrouchToColliders (bool crouched)
		{
			if (_characterController)
			{
				if (crouched)
				{
					_characterController.height = _origControllerHeight * crouchHeightMultiplier;
					_characterController.center = new Vector3(_origControllerCenter.x, _origControllerCenter.y * crouchHeightMultiplier, _origControllerCenter.z);
				}
				else
				{
					_characterController.height = _origControllerHeight;
					_characterController.center = _origControllerCenter;
				}
				return;
			}

			var capsule = GetComponent<CapsuleCollider>();
			if (capsule)
			{
				if (crouched)
				{
					capsule.height = _origCapsuleHeight * crouchHeightMultiplier;
					capsule.center = new Vector3(_origCapsuleCenter.x, _origCapsuleCenter.y * crouchHeightMultiplier, _origCapsuleCenter.z);
				}
				else
				{
					capsule.height = _origCapsuleHeight;
					capsule.center = _origCapsuleCenter;
				}
			}
		}

		private bool HasHeadroomToStand ()
		{
			if (_characterController)
			{
				float radius = _characterController.radius;
				float standHeight = _origControllerHeight;
				Vector3 centerWorld = Transform.TransformPoint(_origControllerCenter);
				Vector3 up = UpDirection.normalized;
				Vector3 bottom = centerWorld - up * (standHeight * 0.5f - radius);
				Vector3 top = centerWorld + up * (standHeight * 0.5f - radius);
				return !Physics.CheckCapsule(bottom, top, radius, ~0, QueryTriggerInteraction.Ignore);
			}

			var capsule = GetComponent<CapsuleCollider>();
			if (capsule)
			{
				float radius = capsule.radius;
				float standHeight = _origCapsuleHeight;
				Vector3 centerWorld = Transform.TransformPoint(_origCapsuleCenter);
				Vector3 up = UpDirection.normalized;
				Vector3 bottom = centerWorld - up * (standHeight * 0.5f - radius);
				Vector3 top = centerWorld + up * (standHeight * 0.5f - radius);
				return !Physics.CheckCapsule(bottom, top, radius, ~0, QueryTriggerInteraction.Ignore);
			}

			return true;
		}


		#endregion
	}
}

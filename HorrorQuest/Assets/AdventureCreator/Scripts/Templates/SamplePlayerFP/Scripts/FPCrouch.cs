using UnityEngine;

namespace AC.Templates.FirstPersonPlayer
{
    [RequireComponent(typeof(AC.Player))]
    [RequireComponent(typeof(CharacterController))]
    public class FPCrouch : MonoBehaviour
    {
        [Header("Input")]
        public string inputButton = "Crouch";

        [Header("Refs")]
        public Transform cameraParent;
        public CharacterController characterController;
        public LayerMask standLayerMask = ~0;

        [Header("Crouch")]
        [Range(0.1f, 0.9f)] public float speedReduction = 0.6f;
        [Range(0.1f, 0.9f)] public float heightReduction = 0.6f;
        public float transitionSpeed = 5f;
        public bool preventRunning = true;

        [Header("Anim")]
        public Animator animator = null;
        public string isCrouchingBool = "";

        private AC.Player player;
        private bool isCrouching;
        private float normalWalkSpeed;
        private float normalRunSpeed;

        private float radius;
        private float standingHeight;
        private float cameraStandingLocalY;
        private float targetHeight;
        private float targetCameraLocalY;

        private Vector3 originalCenter;

        void Awake()
        {
            if (!characterController) characterController = GetComponent<CharacterController>();
            if (!cameraParent)
            {
                var cam = Camera.main;
                if (cam != null)
                    cameraParent = cam.transform.parent != null ? cam.transform.parent : cam.transform;
            }
        }

        void Start()
        {
            player = GetComponent<AC.Player>();
            normalWalkSpeed = player.walkSpeedScale;
            normalRunSpeed  = player.runSpeedScale;
            radius          = characterController.radius;
            standingHeight  = characterController.height;
            originalCenter  = characterController.center;
            if (cameraParent) cameraStandingLocalY = cameraParent.localPosition.y;
            Stand(true);
        }

        void Update()
        {
            if (KickStarter.stateHandler.IsInGameplay() &&
                KickStarter.playerInput.InputGetButtonDown(inputButton))
            {
                if (isCrouching) Stand(false);
                else Crouch();
            }

            if (cameraParent)
            {
                float newY = Mathf.MoveTowards(
                    cameraParent.localPosition.y,
                    targetCameraLocalY,
                    transitionSpeed * Time.deltaTime * Mathf.Abs(targetCameraLocalY - cameraParent.localPosition.y) + 0.0001f
                );
                var lp = cameraParent.localPosition;
                cameraParent.localPosition = new Vector3(lp.x, newY, lp.z);
            }

            float newHeight = Mathf.MoveTowards(characterController.height, targetHeight, transitionSpeed * Time.deltaTime);
            float heightDelta = newHeight - standingHeight;
            characterController.height = newHeight;
            characterController.center = new Vector3(originalCenter.x, originalCenter.y + heightDelta * 0.5f, originalCenter.z);
        }

        public void Stand() => Stand(true);

        private void Crouch(bool force = false)
        {
            if (force || CanCrouch())
            {
                isCrouching = true;
                player.walkSpeedScale = normalWalkSpeed * speedReduction;
                player.runSpeedScale  = normalRunSpeed  * speedReduction;
                targetHeight        = standingHeight * heightReduction;
                targetCameraLocalY  = cameraStandingLocalY * heightReduction;
                if (preventRunning) player.runningLocked = PlayerMoveLock.AlwaysWalk;
                if (animator && !string.IsNullOrEmpty(isCrouchingBool)) animator.SetBool(isCrouchingBool, true);
            }
        }

        private void Stand(bool force)
        {
            if (force || CanStand())
            {
                isCrouching = false;
                player.walkSpeedScale = normalWalkSpeed;
                player.runSpeedScale  = normalRunSpeed;
                targetHeight       = standingHeight;
                targetCameraLocalY = cameraStandingLocalY;
                if (preventRunning) player.runningLocked = PlayerMoveLock.Free;
                if (animator && !string.IsNullOrEmpty(isCrouchingBool)) animator.SetBool(isCrouchingBool, false);
            }
        }

        private bool CanStand()
        {
            int mask = standLayerMask & ~(1 << gameObject.layer);
            Vector3 bottom = transform.position + Vector3.up * (radius + characterController.skinWidth);
            Vector3 top    = transform.position + Vector3.up * (standingHeight - radius);
            bool blocked = Physics.CheckCapsule(bottom, top, radius * 0.98f, mask, QueryTriggerInteraction.Ignore);
            return !blocked;
        }

        private bool CanCrouch()
        {
            return KickStarter.player.IsGrounded();
        }

#if UNITY_EDITOR
        void OnValidate()
        {
            if (characterController == null) characterController = GetComponent<CharacterController>();
            transitionSpeed = Mathf.Max(0.01f, transitionSpeed);
            heightReduction = Mathf.Clamp(heightReduction, 0.1f, 0.9f);
            speedReduction  = Mathf.Clamp(speedReduction,  0.1f, 0.9f);
        }
#endif
    }
}

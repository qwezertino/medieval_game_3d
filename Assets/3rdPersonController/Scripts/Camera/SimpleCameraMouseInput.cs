using UnityEngine;
using UnityEngine.InputSystem;

/// <summary>
/// Простой скрипт для управления камерой vThirdPersonCamera с помощью мыши.
/// Использует новую Input System.
/// Повесьте этот скрипт на объект персонажа и укажите ссылку на камеру в инспекторе.
/// </summary>
public class SimpleCameraMouseInput : MonoBehaviour
{
    [Header("Camera Reference")]
    [Tooltip("Ссылка на камеру vThirdPersonCamera, которой нужно управлять")]
    public vThirdPersonCamera tpCamera;

    [Header("Mouse Sensitivity")]
    [Tooltip("Чувствительность мыши (умножается на значения из vThirdPersonCamera)")]
    public float mouseSensitivity = 1f;

    [Header("Cursor Settings")]
    [Tooltip("Блокировать курсор в центре экрана")]
    public bool lockCursor = true;

    private bool isCursorLocked;
    void Start()
    {
        InitializeCamera();
        SetCursorLock(lockCursor);
    }

    void Update()
    {
        // Обработка переключения блокировки курсора
        HandleCursorToggle();

        // Обработка ввода мыши для вращения камеры
        HandleCameraInput();
    }

    /// <summary>
    /// Обработка переключения блокировки курсора
    /// </summary>
    private void HandleCursorToggle()
    {
        if (Keyboard.current != null && Keyboard.current.escapeKey.wasPressedThisFrame)
        {
            lockCursor = !lockCursor;
            SetCursorLock(lockCursor);
        }
    }

    /// <summary>
    /// Инициализация камеры
    /// </summary>
    private void InitializeCamera()
    {
        // Если камера не указана вручную, попробуем найти её на сцене
        if (tpCamera == null)
        {
            tpCamera = FindFirstObjectByType<vThirdPersonCamera>();

            if (tpCamera == null)
            {
                Debug.LogWarning("SimpleCameraMouseInput: Камера vThirdPersonCamera не найдена! Пожалуйста, укажите её в инспекторе.");
                return;
            }
        }

        // Устанавливаем персонажа как целевой объект для камеры
        if (tpCamera != null)
        {
            tpCamera.SetMainTarget(this.transform);
        }
    }

    /// <summary>
    /// Обработка ввода мыши для вращения камеры
    /// </summary>
    private void HandleCameraInput()
    {
        if (tpCamera == null)
            return;

        // Получаем движение мыши через новую Input System
        if (Mouse.current != null)
        {
            Vector2 mouseDelta = Mouse.current.delta.ReadValue();

            // Нормализуем значения (новая система возвращает пиксели, старая - нормализованные значения)
            float mouseX = mouseDelta.x * 0.02f * mouseSensitivity;
            float mouseY = mouseDelta.y * 0.02f * mouseSensitivity;

            // Передаём значения в камеру для вращения
            tpCamera.RotateCamera(mouseX, mouseY);
        }
    }

    /// <summary>
    /// Установка состояния блокировки курсора
    /// </summary>
    /// <param name="locked">True - заблокировать, False - разблокировать</param>
    private void SetCursorLock(bool locked)
    {
        isCursorLocked = locked;

        if (locked)
        {
            Cursor.lockState = CursorLockMode.Locked;
            Cursor.visible = false;
        }
        else
        {
            Cursor.lockState = CursorLockMode.None;
            Cursor.visible = true;
        }
    }

    /// <summary>
    /// Обработка фокуса приложения
    /// </summary>
    private void OnApplicationFocus(bool hasFocus)
    {
        if (hasFocus && lockCursor)
        {
            SetCursorLock(true);
        }
    }

    /// <summary>
    /// Позволяет программно установить камеру
    /// </summary>
    public void SetCamera(vThirdPersonCamera camera)
    {
        tpCamera = camera;
        if (tpCamera != null)
        {
            tpCamera.SetMainTarget(this.transform);
        }
    }

    /// <summary>
    /// Получить текущую камеру
    /// </summary>
    public vThirdPersonCamera GetCamera()
    {
        return tpCamera;
    }
}

using System.IO;
using UnityEngine;
 
public class CaptureImages : MonoBehaviour
{
    public KeyCode screenshotKey;
    private Camera _camera;
 
    void Start () {
      _camera = GetComponent<Camera>();
    }
 
    private void LateUpdate()
    {
        if (Input.GetKeyDown(screenshotKey))
        {
            Debug.Log("Capture !!!");
            Capture();
        }
    }
 
    public void Capture()
    {
        RenderTexture activeRenderTexture = RenderTexture.active;
        Debug.Log(_camera);
        RenderTexture.active = _camera.targetTexture;
 
        _camera.Render();
 
        Texture2D image = new Texture2D(_camera.targetTexture.width, _camera.targetTexture.height);
        image.ReadPixels(new Rect(0, 0, _camera.targetTexture.width, _camera.targetTexture.height), 0, 0);
        image.Apply();
        RenderTexture.active = activeRenderTexture;
 
        byte[] bytes = image.EncodeToPNG();
        Destroy(image);
 
        Debug.Log(bytes);
        Debug.Log("Captrued Done !!!");
 
        File.WriteAllBytes(Path.Combine(Application.dataPath, "output.png"), bytes);
    }
}
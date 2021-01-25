package orb.slam.test;

import androidx.appcompat.app.AppCompatActivity;

import android.annotation.SuppressLint;
import android.app.ActionBar;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.ImageFormat;
import android.graphics.Matrix;
import android.graphics.Rect;
import android.graphics.YuvImage;
import android.hardware.Camera;
import android.os.Bundle;
import android.os.Handler;
import android.util.Log;
import android.view.MotionEvent;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.view.View;
import android.view.ViewGroup;
import android.widget.ImageView;
import android.widget.TextView;
import android.widget.Toast;

import com.yanzhenjie.permission.AndPermission;
import com.yanzhenjie.permission.runtime.Permission;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.util.List;

public class MainActivity extends AppCompatActivity {
    SurfaceView surface_view;
    ImageView iv_preview;

    private SurfaceHolder surfaceHolder;
    private Camera camera;
    private Camera.PreviewCallback callback_camera;

    /**
     * Load Native Lib
     */
    static {
        System.loadLibrary("native-lib");
    }

    /**
     * Native Function
     * @return
     */
    public native String stringFromJNI();
    public native byte[] transData(byte[] data, int len, int width, int height);

    @SuppressLint("WrongConstant")
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        TextView tv = findViewById(R.id.sample_text);
        tv.setText(stringFromJNI());

        init();
        surfaceHolder.addCallback(callback);
        AndPermission.with(this)
                .runtime()
                .permission(Permission.Group.CAMERA)
                .onGranted(permissions -> {
                    Toast.makeText(this, "获取权限失败", Toast.LENGTH_SHORT).show();
                })
                .onDenied(permissions -> {
                    Toast.makeText(this, "获取权限成功,请重启应用", Toast.LENGTH_SHORT).show();

                })
                .start();
    }

    @SuppressLint("ClickableViewAccessibility")
    private void init() {
        iv_preview = findViewById(R.id.iv_preview);
        surface_view = findViewById(R.id.surface_view);
        surface_view.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                if (camera != null)
                    camera.autoFocus(new Camera.AutoFocusCallback() {
                        @Override
                        public void onAutoFocus(boolean success, Camera camera) {
                        }
                    });
                return false;
            }
        });
        surfaceHolder = surface_view.getHolder();
        surfaceHolder.setKeepScreenOn(true);
        surfaceHolder.setType(SurfaceHolder.SURFACE_TYPE_PUSH_BUFFERS);
        callback_camera = new Camera.PreviewCallback() {
            @Override
            public void onPreviewFrame(byte[] data, Camera camera) {
                Camera.Size size = camera.getParameters().getPreviewSize();
                byte[] data_result = transData(data, data.length, size.width, size.height);
                Bitmap map = transDataToBitmap(data_result);
                iv_preview.setImageBitmap(map);
            }
        };
    }

    private Bitmap transDataToBitmap(byte[] data) {
        long time = System.currentTimeMillis();
        byte[] rawImage;
        ByteArrayOutputStream baos;
        Camera.Size previewSize = camera.getParameters().getPreviewSize();
        BitmapFactory.Options newOpts = new BitmapFactory.Options();
        newOpts.inJustDecodeBounds = true;
        YuvImage yuvimage = new YuvImage(
                data,
                ImageFormat.NV21,
                previewSize.width,
                previewSize.height,
                null);
        baos = new ByteArrayOutputStream();
        yuvimage.compressToJpeg(new Rect(0, 0, previewSize.width, previewSize.height), 100, baos);// 80--JPG图片的质量[0-100],100最高
        rawImage = baos.toByteArray();
        BitmapFactory.Options options = new BitmapFactory.Options();
        options.inPreferredConfig = Bitmap.Config.RGB_565;
        Bitmap bitmap = BitmapFactory.decodeByteArray(rawImage, 0, rawImage.length, options);
        Matrix matrix = new Matrix();
        matrix.postRotate(90);
        Bitmap resizedBitmap = Bitmap.createBitmap(bitmap, 0, 0,
                bitmap.getWidth(), bitmap.getHeight(), matrix, true);
        bitmap.recycle();
        bitmap = null;

        time = System.currentTimeMillis() - time;
        Log.i("asdasdawda", time + "");
        return resizedBitmap;
    }


    private void resizeView() {
        ViewGroup.LayoutParams params = surface_view.getLayoutParams();
        int height = (int) (ScreenUtils.getScreenWidth(this) / 600.0f * 800.0f);
        params.height = height;
        surface_view.setLayoutParams(params);
    }

    private SurfaceHolder.Callback callback = new SurfaceHolder.Callback() {
        @Override
        public void surfaceCreated(SurfaceHolder holder) {
            resizeView();
            camera = Camera.open(0);
            Camera.Parameters params = camera.getParameters();
            List<Camera.Size> sizes = params.getSupportedPreviewSizes();
            params.setPreviewSize(800, 600);
            camera.setParameters(params);
            try {
                camera.setDisplayOrientation(90);
                camera.setPreviewDisplay(holder);
            } catch (IOException e) {
                e.printStackTrace();
                Toast.makeText(MainActivity.this, "相机设置预览失败", Toast.LENGTH_SHORT).show();
            }
            camera.setPreviewCallback(callback_camera);
            camera.startPreview();
        }

        @Override
        public void surfaceChanged(SurfaceHolder holder, int format, int width, int height) {
        }

        @Override
        public void surfaceDestroyed(SurfaceHolder holder) {
            camera.stopPreview(); // (一定要有，不然只release也可能出问题)
            camera.release();
            camera = null;
            System.gc();
        }
    };

    @Override
    protected void onDestroy() {
        super.onDestroy();
        if (camera != null) {
            camera.setPreviewCallback(null);
            camera.stopPreview();
            camera.release();
            camera = null;
        }
        surface_view = null;
    }
}
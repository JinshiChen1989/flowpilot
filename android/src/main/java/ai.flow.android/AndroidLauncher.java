package ai.flow.android;

import ai.flow.android.sensor.CameraManager;
import ai.flow.android.sensor.SensorManager;
import ai.flow.android.vision.SNPEModelRunner;
import ai.flow.app.FlowUI;
import ai.flow.common.ParamsInterface;
import ai.flow.common.Path;
import ai.flow.common.transformations.Camera;
import ai.flow.common.utils;
import ai.flow.hardware.HardwareManager;
import ai.flow.launcher.Launcher;
import ai.flow.modeld.*;
import ai.flow.sensor.SensorInterface;
import android.annotation.SuppressLint;
import android.content.Context;
import android.os.Process;
import android.os.*;
import android.provider.Settings;
import android.system.ErrnoException;
import android.system.Os;
import android.telephony.TelephonyManager;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Toast;
import androidx.fragment.app.FragmentActivity;
import androidx.fragment.app.FragmentTransaction;
import com.badlogic.gdx.backends.android.AndroidApplicationConfiguration;
import com.badlogic.gdx.backends.android.AndroidFragmentApplication;
import org.acra.ACRA;
import org.acra.ErrorReporter;
import org.jetbrains.annotations.NotNull;

import java.util.*;


/** Launches the main android flowpilot application. */
public class AndroidLauncher extends FragmentActivity implements AndroidFragmentApplication.Callbacks {
	public static Map<String, SensorInterface> sensors;
	public static Context appContext;
	public static ParamsInterface params;

	@SuppressLint("HardwareIds")
	@Override
	protected void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		appContext = getApplicationContext();

		// set environment variables from intent extras.
		Bundle bundle = getIntent().getExtras();
		if (bundle != null) {
			for (String key : bundle.keySet()) {
				if (bundle.get(key) == null)
					continue;
				try {
					Os.setenv(key, (String)bundle.get(key), true);
				} catch (Exception ignored) {
				}
			}
		}

		try {
			Os.setenv("USE_GPU", "1", true);
		} catch (ErrnoException e) {
			throw new RuntimeException(e);
		}


		HardwareManager androidHardwareManager = new AndroidHardwareManager(getWindow());
		// keep app from dimming due to inactivity.
		androidHardwareManager.enableScreenWakeLock(true);

		// get wakelock so we can switch windows without getting killed.
		PowerManager powerManager = (PowerManager) getSystemService(POWER_SERVICE);
		PowerManager.WakeLock wakeLock = powerManager.newWakeLock(PowerManager.PARTIAL_WAKE_LOCK, "ai.flow.app::wakelock");

		// acquiring wakelock causes crash on some devices.
		//try {
		//	wakeLock.acquire();
		//} catch (Exception e){
		//	System.err.println(e);
		//}

		// tune system for max throughput. Does this really help ?
		//if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.N) {
		//	getWindow().setSustainedPerformanceMode(true);
		//}

		params = ParamsInterface.getInstance();
		//utils.F3Mode = true; //params.existsAndCompare("F3", true);
		//utils.WideCameraOnly = true; //params.existsAndCompare("WideCameraOnly", true);

		TelephonyManager telephonyManager = (TelephonyManager)getSystemService(Context.TELEPHONY_SERVICE);
		String dongleID = "";
		if (telephonyManager != null) {
			dongleID = Settings.Secure.getString(appContext.getContentResolver(), Settings.Secure.ANDROID_ID);
		}

		// populate device specific info.
		params.put("DongleId", dongleID);
		params.put("DeviceManufacturer", Build.MANUFACTURER);
		params.put("DeviceModel", Build.MODEL);

		AndroidApplicationConfiguration configuration = new AndroidApplicationConfiguration();
		CameraManager cameraManager, cameraManagerWide = null;
		SensorManager sensorManager = new SensorManager(appContext, 100);
		if (utils.SingleCameraOnly) {
			cameraManager = new CameraManager(getApplication().getApplicationContext(), 20, Camera.CAMERA_TYPE_WIDE);
			CameraManager finalCameraManager = cameraManager; // stupid java
			sensors = new HashMap<String, SensorInterface>() {{
				put("roadCamera", finalCameraManager);
				put("wideRoadCamera", finalCameraManager); // use same camera until we move away from wide camera-only mode.
				put("motionSensors", sensorManager);
			}};
		} else {
			cameraManager = new CameraManager(getApplication().getApplicationContext(), 20, Camera.CAMERA_TYPE_ROAD);
			cameraManagerWide = new CameraManager(getApplication().getApplicationContext(), 20, Camera.CAMERA_TYPE_WIDE);
			CameraManager finalCameraManager = cameraManager; // stupid java
			CameraManager finalCameraManagerWide = cameraManagerWide;
			sensors = new HashMap<String, SensorInterface>() {{
				put("roadCamera", finalCameraManager);
				put("wideRoadCamera", finalCameraManagerWide);
				put("motionSensors", sensorManager);
			}};
		}

		int pid = Process.myPid();

		String modelPath = Path.getModelDir();

		ModelRunner model;
		boolean useGPU = true; // always use gpus on android phones.
		if (params.getBool("UseSNPE"))
			model = new SNPEModelRunner(getApplication(), modelPath, useGPU);
		else
			model = new TNNModelRunner(modelPath, useGPU);

		ModelExecutor modelExecutor;
		modelExecutor = new ModelExecutorF3(model);
		Launcher launcher = new Launcher(sensors, modelExecutor);

		ErrorReporter ACRAreporter = ACRA.getErrorReporter();
		ACRAreporter.putCustomData("DongleId", dongleID);
		ACRAreporter.putCustomData("AndroidAppVersion", ai.flow.app.BuildConfig.VERSION_NAME);
		ACRAreporter.putCustomData("FlowpilotVersion", params.getString("Version"));
		ACRAreporter.putCustomData("VersionMisMatch", checkVersionMisMatch().toString());

		ACRAreporter.putCustomData("GitCommit", params.getString("GitCommit"));
		ACRAreporter.putCustomData("GitBranch", params.getString("GitBranch"));
		ACRAreporter.putCustomData("GitRemote", params.getString("GitRemote"));

		MainFragment fragment = new MainFragment(new FlowUI(launcher, androidHardwareManager, pid));
		cameraManager.setLifeCycleFragment(fragment);
		if (cameraManagerWide != null) cameraManagerWide.setLifeCycleFragment(fragment);
		FragmentTransaction trans = getSupportFragmentManager().beginTransaction();
		trans.replace(android.R.id.content, fragment);
		trans.commit();
	}

	public static class MainFragment extends AndroidFragmentApplication {
		FlowUI flowUI;

		MainFragment(FlowUI flowUI) {
			this.flowUI = flowUI;
		}

		@Override
		public View onCreateView(@NotNull LayoutInflater inflater, ViewGroup container, Bundle savedInstanceState) {
			return initializeForView(flowUI);
		}
	}

	@Override
	protected void attachBaseContext(Context base) {
		super.attachBaseContext(base);
	}

	private Boolean checkVersionMisMatch() {
		// check version mismatch between android app and github repo project.
		if (!params.getString("Version").equals(ai.flow.app.BuildConfig.VERSION_NAME)) {
			Toast.makeText(appContext, "WARNING: App version mismatch detected. Make sure you are using compatible versions of apk and github repo.", Toast.LENGTH_LONG).show();
			return true;
		}
		return false;
	}

	@Override
	public void exit() {
	}

	@Override
	public void onBackPressed() {
		return;
	}
}


#include <jni.h>
#include <android/log.h>
#include <algorithm>

#include "band.h"

static jclass bdata_class;
static jclass gdata_class;
static jclass oom_error;
static jclass null_error;
static jclass invstate_error;

static jfieldID bcal_handle_id;
static jfieldID bdet_handle_id;
static jmethodID bdata_init_id;
static jfieldID bdata_timestamp_id;
static jmethodID gdata_init_id;
static const char *const bdata_field_name[] = { "gx", "gy", "gz", "ax", "ay", "az" };
static constexpr size_t bdata_field_count = sizeof(bdata_field_name)/sizeof(*bdata_field_name);
static jfieldID bdata_field_id[bdata_field_count];

template <typename T>
static void wrap_create(JNIEnv* env, jobject self, jfieldID handle_id)
{
	__android_log_print(ANDROID_LOG_DEBUG, "BandSupportLib", "%s/create()", typeid(T).name());
	if (env->GetLongField(self, handle_id) != 0) {
		env->ThrowNew(invstate_error, "Non-null handle");
		return;
	}

	std::unique_ptr<T> p;

	try {
		p = T::create();
		env->SetLongField(self, handle_id, (uintptr_t)p.get());
	} catch (...) {
		env->ThrowNew(oom_error, "Native error in constructor");	// FIXME
	}

	__android_log_print(ANDROID_LOG_DEBUG, "BandSupportLib", "create: %s @ %p%s",
		typeid(T).name(), p.get(), env->ExceptionCheck() ? "[throwing]" : "");

	if (!env->ExceptionCheck())
		(void)p.release();
}

extern "C" JNIEXPORT void JNICALL
Java_com_omnichip_gameinn_bandgame_BandCalibrator_create(JNIEnv* env, jobject self)
{
	wrap_create<BandCalibrator>(env, self, bcal_handle_id);
}

static BandCalibrator *get_handle(JNIEnv *env, jobject self)
{
	auto p = reinterpret_cast<BandCalibrator *>(uintptr_t(env->GetLongField(self, bcal_handle_id)));
	if (!p)
		env->ThrowNew(invstate_error, "Null handle");
	__android_log_print(ANDROID_LOG_DEBUG, "BandSupportLib", "handle: BandCalibrator @ %p", p);
	return p;
}

extern "C" JNIEXPORT void JNICALL
Java_com_omnichip_gameinn_bandgame_BandCalibrator_destroy(JNIEnv* env, jobject self)
{
	BandCalibrator *me = get_handle(env, self);
	__android_log_print(ANDROID_LOG_DEBUG, "BandSupportLib", "destroy: BandCalibrator @ %p", me);
	env->SetLongField(self, bcal_handle_id, 0);
	delete me;
}

static bool unpack_sensor_data(BandDevice::SensorData& sd, JNIEnv* env, jobject data)
{
	if (!data) {
		env->ThrowNew(null_error, "Null data");
		return false;
	}

	sd.timestamp = unsigned(env->GetLongField(data, bdata_timestamp_id));
	auto *p = &sd.v.gx;
	for (auto& fid : bdata_field_id)
		*p++ = env->GetIntField(data, fid);

	return true;
}

extern "C" JNIEXPORT jboolean JNICALL
Java_com_omnichip_gameinn_bandgame_BandCalibrator_process(JNIEnv* env, jobject self, jobject data)
{
	BandCalibrator *me = get_handle(env, self);
	if (!me)
		return false;

	BandDevice::SensorData sd;
	if (!unpack_sensor_data(sd, env, data))
		return false;

	return me->process(sd);
}

extern "C" JNIEXPORT jobject JNICALL
Java_com_omnichip_gameinn_bandgame_BandCalibrator_getZeroOffset(JNIEnv* env, jobject self)
{
	BandCalibrator *me = get_handle(env, self);
	if (!me)
		return nullptr;

	jobject data = env->NewObject(bdata_class, bdata_init_id);
	if (!data)
		return nullptr;

	auto sd = me->zero_offset();
	auto *p = &sd.gx;
	for (auto& fid : bdata_field_id)
		env->SetIntField(data, fid, *p++);

	return data;
}

extern "C" JNIEXPORT jboolean JNICALL
Java_com_omnichip_gameinn_bandgame_BandCalibrator_isDone(JNIEnv* env, jobject self)
{
	BandCalibrator *me = get_handle(env, self);
	if (!me)
		return false;

	return me->is_done();
}

extern "C" JNIEXPORT jbyte JNICALL
Java_com_omnichip_gameinn_bandgame_BandCalibrator_getCalibrationNeededMask(JNIEnv* env, jobject self)
{
	BandCalibrator *me = get_handle(env, self);
	if (!me)
		return false;

	return me->calibration_needed_mask();
}

extern "C" JNIEXPORT jstring JNICALL
Java_com_omnichip_gameinn_bandgame_BandCalibrator_getStatusString(JNIEnv* env, jobject self)
{
	BandCalibrator *me = get_handle(env, self);
	if (!me)
		return nullptr;

	return env->NewStringUTF(me->status().c_str());
}

extern "C" JNIEXPORT void JNICALL
Java_com_omnichip_gameinn_bandgame_BandDetector_create(JNIEnv* env, jobject self)
{
	wrap_create<BandDetector>(env, self, bdet_handle_id);
}

static BandDetector *get_detector(JNIEnv *env, jobject self)
{
	auto p = reinterpret_cast<BandDetector *>(uintptr_t(env->GetLongField(self, bdet_handle_id)));
	if (!p)
		env->ThrowNew(invstate_error, "Null handle");
	return p;
}

extern "C" JNIEXPORT void JNICALL
Java_com_omnichip_gameinn_bandgame_BandDetector_destroy(JNIEnv* env, jobject self)
{
	BandDetector *me = get_detector(env, self);
	__android_log_print(ANDROID_LOG_DEBUG, "BandSupportLib", "BandDetector/destroy(%s)", me ? "" : "NULL");
	env->SetLongField(self, bdet_handle_id, 0);
	delete me;
}

extern "C" JNIEXPORT jlong JNICALL
Java_com_omnichip_gameinn_bandgame_BandDetector_getGestures(JNIEnv* env, jobject self, jint mask)
{
	BandDetector *me = get_detector(env, self);
	if (!me)
		return 0;

	BandDetector::gesture_bitmap_t bm = me->get_gestures((unsigned)mask);
	return bm.to_ullong();
}

extern "C" JNIEXPORT void JNICALL
Java_com_omnichip_gameinn_bandgame_BandDetector_enableGestures(JNIEnv* env, jobject self, jlong mask)
{
	BandDetector *me = get_detector(env, self);
	if (!me)
		return;

	try {
		BandDetector::gesture_bitmap_t bm{(unsigned long long)mask};
		me->enable_gestures(bm);
	} catch (const std::invalid_argument& e) {
		env->ThrowNew(invstate_error, "Bad band enable mask");
	}
}

static jobject pack_event_data(JNIEnv* env, const BandDetector::GestureData& g)
{
	jobject jev = env->NewObject(gdata_class, gdata_init_id,
		(jlong)g.timestamp, (jint)g.type,
		(jfloat)g.value.x, (jfloat)g.value.y, (jfloat)g.value.z);
	if (!jev)
		env->ThrowNew(oom_error, "No memoy for gesture data");
	return jev;
}

extern "C" JNIEXPORT jobjectArray JNICALL
Java_com_omnichip_gameinn_bandgame_BandDetector_process(JNIEnv* env, jobject self, jint which, jobject data)
{
	BandDetector *me = get_detector(env, self);
	if (!me)
		return nullptr;

	if (which < 0 || which > BandDetector::__NUM_LOCATIONS) {
		env->ThrowNew(invstate_error, "Bad band location ID");
		return nullptr;
	}

	BandDevice::SensorData sd;
	if (!unpack_sensor_data(sd, env, data))
		return nullptr;

	auto evs = me->process((BandDetector::BandLocation)which, sd);
	jobjectArray jevs = env->NewObjectArray(evs.size(), gdata_class, nullptr);
	if (!jevs)
		return nullptr;

	size_t i = 0;
	for (const auto& g : evs) {
		jobject ev = pack_event_data(env, g);
		if (!ev)
			return nullptr;
		env->SetObjectArrayElement(jevs, i++, ev);
		env->DeleteLocalRef(ev);
	}

	return jevs;
}

extern "C" JNIEXPORT void JNICALL
Java_com_omnichip_gameinn_bandgame_BandDetector_reset(JNIEnv* env, jobject self)
{
	BandDetector *me = get_detector(env, self);
	if (!me)
		return;

	try {
		me->reset();
	} catch (...) {
		env->ThrowNew(invstate_error, "BUG: reset() threw");
	}
}

template <typename TR, typename... Args>
static TR check_nonnull(TR ptr, const char *err_fmt, Args&&... args)
{
	if (ptr)
		return ptr;
	__android_log_print(ANDROID_LOG_FATAL, "BandSupportLib", err_fmt, std::forward<Args>(args)...);
	throw 0;
}

extern "C" JNIEXPORT jint JNICALL
JNI_OnLoad(JavaVM* vm, void* reserved) {
	JNIEnv* env;
	if (vm->GetEnv(reinterpret_cast<void**>(&env), JNI_VERSION_1_6) != JNI_OK) {
		return JNI_ERR;
	}

	// Find your class. JNI_OnLoad is called from the correct class loader context for this to work.
	try {
		auto init_class = [env] (const char *name, bool retain) {
			auto cls = check_nonnull(env->FindClass(name), "class not found: %s", name);
			if (retain)
				cls = check_nonnull(static_cast<jclass>(env->NewGlobalRef(cls)), "can't get global ref to class object for %s", name);
			return cls;
		};

		auto init_field = [env] (jclass cls, const char *name, const char *type) {
			return check_nonnull(env->GetFieldID(cls, name, type), "field not found: %s %s", name, type);
		};

		auto init_method = [env] (jclass cls, const char *name, const char *sig) {
			return check_nonnull(env->GetMethodID(cls, name, sig), "method not found: %s %s", name, sig);
		};

		auto bcal_class = init_class("com/omnichip/gameinn/bandgame/BandCalibrator", false);
		auto bdet_class = init_class("com/omnichip/gameinn/bandgame/BandDetector", false);
		bdata_class = init_class("com/omnichip/gameinn/bandgame/BandData", true);
		gdata_class = init_class("com/omnichip/gameinn/bandgame/GestureData", true);
		oom_error = init_class("java/lang/OutOfMemoryError", true);
		null_error = init_class("java/lang/NullPointerException", true);
		invstate_error = init_class("java/lang/IllegalStateException", true);

		bcal_handle_id = init_field(bcal_class, "handle", "J");
		bdet_handle_id = init_field(bdet_class, "handle", "J");
		bdata_init_id = init_method(bdata_class, "<init>", "()V");
		bdata_timestamp_id = init_field(bdata_class, "timestamp", "J");
		std::transform(bdata_field_name, bdata_field_name + bdata_field_count, bdata_field_id, [&init_field] (const char *name) {
			return init_field(bdata_class, name, "I");
		});
		gdata_init_id = init_method(gdata_class, "<init>", "(JIFFF)V");
	} catch (int) {
		return JNI_ERR;
	}

	return JNI_VERSION_1_6;
}

#import <Foundation/Foundation.h>

#include <dispatch/dispatch.h>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <dlfcn.h>

struct Il2CppDomain;
struct Il2CppAssembly;
struct Il2CppImage;
struct Il2CppClass;
struct Il2CppObject;
struct FieldInfo;

namespace {

using DomainGetFn = Il2CppDomain* (*)();
using ThreadAttachFn = void* (*)(Il2CppDomain*);
using DomainGetAssembliesFn = const Il2CppAssembly** (*)(Il2CppDomain*, size_t*);
using AssemblyGetImageFn = const Il2CppImage* (*)(const Il2CppAssembly*);
using ImageGetNameFn = const char* (*)(const Il2CppImage*);
using ClassFromNameFn = Il2CppClass* (*)(const Il2CppImage*, const char*, const char*);
using ClassGetFieldFn = FieldInfo* (*)(Il2CppClass*, const char*);
using FieldStaticGetFn = void (*)(FieldInfo*, void*);
using FieldGetFn = void (*)(Il2CppObject*, FieldInfo*, void*);
using FieldSetFn = void (*)(Il2CppObject*, FieldInfo*, void*);

struct Api {
    DomainGetFn domainGet = nullptr;
    ThreadAttachFn threadAttach = nullptr;
    DomainGetAssembliesFn domainGetAssemblies = nullptr;
    AssemblyGetImageFn assemblyGetImage = nullptr;
    ImageGetNameFn imageGetName = nullptr;
    ClassFromNameFn classFromName = nullptr;
    ClassGetFieldFn classGetField = nullptr;
    FieldStaticGetFn fieldStaticGet = nullptr;
    FieldGetFn fieldGet = nullptr;
    FieldSetFn fieldSet = nullptr;

    template <typename T>
    static T resolve(const char* name) {
        return reinterpret_cast<T>(dlsym(RTLD_DEFAULT, name));
    }

    bool ready() const {
        return domainGet && threadAttach && domainGetAssemblies &&
               assemblyGetImage && imageGetName && classFromName &&
               classGetField && fieldStaticGet && fieldGet && fieldSet;
    }

    bool resolveAll() {
        domainGet = resolve<DomainGetFn>("il2cpp_domain_get");
        threadAttach = resolve<ThreadAttachFn>("il2cpp_thread_attach");
        domainGetAssemblies = resolve<DomainGetAssembliesFn>(
            "il2cpp_domain_get_assemblies");
        assemblyGetImage = resolve<AssemblyGetImageFn>("il2cpp_assembly_get_image");
        imageGetName = resolve<ImageGetNameFn>("il2cpp_image_get_name");
        classFromName = resolve<ClassFromNameFn>("il2cpp_class_from_name");
        classGetField = resolve<ClassGetFieldFn>(
            "il2cpp_class_get_field_from_name");
        fieldStaticGet = resolve<FieldStaticGetFn>("il2cpp_field_static_get_value");
        fieldGet = resolve<FieldGetFn>("il2cpp_field_get_value");
        fieldSet = resolve<FieldSetFn>("il2cpp_field_set_value");
        return ready();
    }
};

struct NativeReflectState {
    Api api;
    Il2CppDomain* attachedDomain = nullptr;
    Il2CppClass* gameInfoClass = nullptr;
    Il2CppClass* settingDataClass = nullptr;
    FieldInfo* gameInfoInstance = nullptr;
    FieldInfo* gameInfoSetting = nullptr;
    FieldInfo* reflectCount = nullptr;
    bool loggedSuccess = false;

    void clearMetadata() {
        gameInfoClass = settingDataClass = nullptr;
        gameInfoInstance = gameInfoSetting = reflectCount = nullptr;
    }

    bool attach() {
        Il2CppDomain* domain = api.domainGet ? api.domainGet() : nullptr;
        if (!domain) return false;
        if (attachedDomain != domain) {
            if (!api.threadAttach || !api.threadAttach(domain)) return false;
            attachedDomain = domain;
            clearMetadata();
        }
        return true;
    }

    const Il2CppImage* pocketImage() const {
        if (!attachedDomain) return nullptr;
        size_t count = 0;
        const Il2CppAssembly** assemblies =
            api.domainGetAssemblies(attachedDomain, &count);
        if (!assemblies || count == 0 || count > 4096) return nullptr;
        for (size_t index = 0; index < count; ++index) {
            const Il2CppImage* image = api.assemblyGetImage(assemblies[index]);
            const char* name = image ? api.imageGetName(image) : nullptr;
            if (name && (std::strcmp(name, "Pocket.Main.dll") == 0 ||
                         std::strcmp(name, "Pocket.Main") == 0)) {
                return image;
            }
        }
        return nullptr;
    }

    bool prepareMetadata() {
        if (gameInfoInstance && gameInfoSetting && reflectCount) return true;
        const Il2CppImage* image = pocketImage();
        if (!image) return false;
        gameInfoClass = api.classFromName(image, "", "GameInfo");
        settingDataClass = api.classFromName(image, "", "SettingData");
        if (!gameInfoClass || !settingDataClass) return false;
        gameInfoInstance = api.classGetField(gameInfoClass, "instance");
        gameInfoSetting = api.classGetField(gameInfoClass, "Setting");
        reflectCount = api.classGetField(settingDataClass, "pocketCueReflectNum");
        return gameInfoInstance && gameInfoSetting && reflectCount;
    }

    bool apply() {
        if (!api.ready() && !api.resolveAll()) return false;
        if (!attach() || !prepareMetadata()) return false;
        Il2CppObject* game = nullptr;
        api.fieldStaticGet(gameInfoInstance, &game);
        if (!game) return false;
        Il2CppObject* setting = nullptr;
        api.fieldGet(game, gameInfoSetting, &setting);
        if (!setting) return false;
        int32_t value = 0;
        api.fieldGet(setting, reflectCount, &value);
        if (value < 0 || value > 64) return false;
        if (value != 3) {
            value = 3;
            api.fieldSet(setting, reflectCount, &value);
        }
        value = 0;
        api.fieldGet(setting, reflectCount, &value);
        if (value == 3 && !loggedSuccess) {
            loggedSuccess = true;
            NSLog(@"[PoolTrajectoryHybrid 0.3.0] native reflect count = 3");
        }
        return value == 3;
    }
};

static NativeReflectState* gNativeReflectState = nullptr;

static void scheduleReflectTick() {
    dispatch_after(dispatch_time(DISPATCH_TIME_NOW,
                                 static_cast<int64_t>(0.5 * NSEC_PER_SEC)),
                   dispatch_get_main_queue(), ^{
        if (!gNativeReflectState) return;
        gNativeReflectState->apply();
        scheduleReflectTick();
    });
}

} // namespace

extern "C" __attribute__((visibility("default"))) void NativeReflectInit(void) {
    dispatch_async(dispatch_get_main_queue(), ^{
        if (gNativeReflectState) return;
        gNativeReflectState = new NativeReflectState();
        gNativeReflectState->apply();
        scheduleReflectTick();
    });
}

__attribute__((constructor)) static void NativeReflectConstructor(void) {
    NativeReflectInit();
}

plugins {
    alias(libs.plugins.android.library) apply false
    alias(libs.plugins.dokka) apply false
    alias(libs.plugins.deployer) apply false
    alias(libs.plugins.kotlin.android) apply false
}

subprojects {
    group = "com.pedropathing"
    version = property("version") as String
}

if (System.getenv("PUBLISH_PEDRO") == "yes please") {
    tasks.register("deployCentralPortal") {
        group = "publishing"
        description = "Publishes all subprojects to Maven Central."
        dependsOn(subprojects.map { it.tasks.named("deployCentralPortal") })
    }

    tasks.register("deployNexusSnapshot") {
        group = "publishing"
        description = "Publishes all subprojects to Maven Central Snapshots."
        dependsOn(subprojects.map { it.tasks.named("deployNexusSnapshot") })
    }
}

tasks.register("deployLocal") {
    group = "publishing"
    description = "Publishes all subprojects to Maven Local."
    dependsOn(subprojects.map { it.tasks.named("deployLocal") })
}plugins {
    id("com.android.library")
    id("org.jetbrains.dokka")
    id("io.deepmedia.tools.deployer")
    kotlin("android")
}

android {
    namespace = "com.pedropathing.ftc"
    compileSdk = 30

    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_1_8
        targetCompatibility = JavaVersion.VERSION_1_8
    }

    publishing {
        singleVariant("release") {
            withSourcesJar()
        }
    }

    defaultConfig {
        minSdk = 21
    }
}

dependencies {
    compileOnly(libs.bundles.ftc)
    api(project(":core"))
    dokkaPlugin(libs.dokka.java.plugin)
}

val dokkaJar = tasks.register<Jar>("dokkaJar") {
    dependsOn(tasks.named("dokkaGenerate"))
    from(dokka.basePublicationsDirectory.dir("html"))
    archiveClassifier = "html-docs"
}

deployer {
    projectInfo {
        name = "Pedro Pathing FTC"
        description = "A path follower designed to revolutionize autonomous pathing in robotics"
        url = "https://github.com/Pedro-Pathing/PedroPathing"
        scm {
            fromGithub("Pedro-Pathing", "PedroPathing")
        }
        license("BSD 3-Clause License", "https://opensource.org/licenses/BSD-3-Clause")

        developer("Baron Henderson", "baron@pedropathing.com")
        developer("Havish Sripada", "havish@pedropathing.com")
    }

    content {
        androidComponents("release") {
            docs(dokkaJar)
        }
    }

    if (System.getenv("PUBLISH_PEDRO") == "yes please") {
        signing {
            key = secret("MVN_GPG_KEY")
            password = secret("MVN_GPG_PASSWORD")
        }

        centralPortalSpec {
            auth {
                user = secret("SONATYPE_USERNAME")
                password = secret("SONATYPE_PASSWORD")
            }
            allowMavenCentralSync = false
        }

        nexusSpec("snapshot") {
            repositoryUrl = "https://central.sonatype.com/repository/maven-snapshots/"
            auth {
                user = secret("SONATYPE_USERNAME")
                password = secret("SONATYPE_PASSWORD")
            }
        }
    }

    localSpec()
}
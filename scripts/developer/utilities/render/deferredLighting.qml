//
//  deferredLighting.qml
//
//  Created by Sam Gateau on 6/6/2016
//  Copyright 2016 High Fidelity, Inc.
//
//  Distributed under the Apache License, Version 2.0.
//  See the accompanying file LICENSE or https://www.apache.org/licenses/LICENSE-2.0.html
//
import QtQuick 2.5
import QtQuick.Controls 1.4
import "configSlider"

Column {
    spacing: 8


    Row {
        spacing: 8
        Column {
            spacing: 10
            Repeater {
                model: [
                     "Unlit:LightingModel:enableUnlit", 
                     "Shaded:LightingModel:enableShaded", 
                     "Emissive:LightingModel:enableEmissive", 
                     "Lightmap:LightingModel:enableLightmap",
                ]
                CheckBox {
                    text: modelData.split(":")[0]
                    checked: Render.getConfig(modelData.split(":")[1])
                    onCheckedChanged: { Render.getConfig(modelData.split(":")[1])[modelData.split(":")[2]] = checked }
                }
            }
        }


        Column {
            spacing: 10
            Repeater {
                model: [
                     "Scattering:LightingModel:enableScattering",
                     "Diffuse:LightingModel:enableDiffuse",
                     "Specular:LightingModel:enableSpecular",
                     "Albedo:LightingModel:enableAlbedo",
                ]
                CheckBox {
                    text: modelData.split(":")[0]
                    checked: Render.getConfig(modelData.split(":")[1])
                    onCheckedChanged: { Render.getConfig(modelData.split(":")[1])[modelData.split(":")[2]] = checked }
                }
            }
        }

        Column {
            spacing: 10
            Repeater {
                model: [
                     "Ambient:LightingModel:enableAmbientLight",
                     "Directional:LightingModel:enableDirectionalLight",
                     "Point:LightingModel:enablePointLight",
                     "Spot:LightingModel:enableSpotLight" 
                ]
                CheckBox {
                    text: modelData.split(":")[0]
                    checked: Render.getConfig(modelData.split(":")[1])
                    onCheckedChanged: { Render.getConfig(modelData.split(":")[1])[modelData.split(":")[2]] = checked }
                }
            }
        }
    }
    Column {
        spacing: 10 
        Repeater {
            model: [ "Tone Mapping exposure:ToneMapping:exposure:5.0:-5.0"
                          ]
            ConfigSlider {
                    label: qsTr(modelData.split(":")[0])
                    integral: false
                    config: Render.getConfig(modelData.split(":")[1])
                    property: modelData.split(":")[2]
                    max: modelData.split(":")[3]
                    min: modelData.split(":")[4]
            }
        }

        ComboBox {
            currentIndex: 1
            model: ListModel {
                id: cbItems
                ListElement { text: "RGB"; color: "Yellow" }
                ListElement { text: "SRGB"; color: "Green" }
                ListElement { text: "Reinhard"; color: "Yellow" }
                ListElement { text: "Filmic"; color: "White" }
            }
            width: 200
            onCurrentIndexChanged: { Render.getConfig("ToneMapping")["curve"] = currentIndex }
        }
    }
}

